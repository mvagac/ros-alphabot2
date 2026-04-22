#include "alphabot2-diffdrive/alphabot2_diffdrive.hpp"

#include <fstream>
#include <unistd.h>
#include <fcntl.h>


#define CHIPNAME "gpiochip0"

// Define GPIO pins (BCM numbering)
#define AIN1 12
#define AIN2 13

#define BIN1 20
#define BIN2 21


namespace ros2_control_alphabot2	//ros2_control_alphabot2
{

/* ***** HW ZAVISLE ************************************************************ */
int DiffDriveAlphabot2::hw_init()
{
  chip = gpiod_chip_open_by_name(CHIPNAME);
  if (!chip)
  {
    std::cerr << "Failed to open GPIO chip\n";
    return 1;
  }

  // PWM
  auto setup_pwm = [this](const std::string& id) -> int {
      std::string path = "/sys/class/pwm/pwmchip" + id + "/pwm0";
      
      if (access(path.c_str(), F_OK) != 0) {				// export if not already present
          std::ofstream("/sys/class/pwm/pwmchip" + id + "/export") << "0";	// creates pwm0 subfolder
      }
      std::ofstream(path + "/enable") << "0";				// disable to allow period changes
      std::ofstream(path + "/period") << std::to_string(PERIOD_NS);	// set period (>duty_cycle)
      std::ofstream(path + "/duty_cycle") << "0";			// set initial duty cycle
      std::ofstream(path + "/enable") << "1";				// enable
      return open((path + "/duty_cycle").c_str(), O_WRONLY);		// open duty_cycle file (keep open)
  };
  pwm0_duty_fd = setup_pwm("0");
  pwm1_duty_fd = setup_pwm("1");

  if (pwm0_duty_fd < 0 || pwm1_duty_fd < 0)
    return 1;

  // pins
  ain1 = gpiod_chip_get_line(chip, AIN1);
  ain2 = gpiod_chip_get_line(chip, AIN2);
  bin1 = gpiod_chip_get_line(chip, BIN1);
  bin2 = gpiod_chip_get_line(chip, BIN2);

  gpiod_line_request_output(ain1, "motor", 0);
  gpiod_line_request_output(ain2, "motor", 0);
  gpiod_line_request_output(bin1, "motor", 0);
  gpiod_line_request_output(bin2, "motor", 0);
  return 0;
}

void DiffDriveAlphabot2::hw_release()
{
  // pwm
  if (pwm0_duty_fd >= 0) {
    lseek(pwm0_duty_fd, 0, SEEK_SET);
    dprintf(pwm0_duty_fd, "0");
    close(pwm0_duty_fd);
  }
  if (pwm1_duty_fd >= 0) {
    lseek(pwm1_duty_fd, 0, SEEK_SET);
    dprintf(pwm1_duty_fd, "0");
    close(pwm1_duty_fd);
  }

  gpiod_chip_close(chip);
}

void DiffDriveAlphabot2::hw_write(char dir, int pwm, std::stringstream &ss)
{
  struct gpiod_line *in1 = NULL;
  struct gpiod_line *in2 = NULL;
  int fd = -1;
  if (dir == 'L')
  {
    in1 = ain1;
    in2 = ain2;
    fd = pwm0_duty_fd;
  }
  if (dir == 'P')
  {
    in1 = bin1;
    in2 = bin2;
    fd = pwm1_duty_fd;
  }
  if (fd == -1)
  {
    return;
  }

  if (pwm == 0)
  {
    // stoj
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t\tstoj " << dir << ", " << pwm;
    gpiod_line_set_value(in1, 0);
    gpiod_line_set_value(in2, 0);
    lseek(fd, 0, SEEK_SET);
    dprintf(fd, "%d", pwm);
  } else if (pwm > 0)
  {
    // dopredu
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t\tdopredu " << dir << ", " << pwm;
    gpiod_line_set_value(in1, 0);
    gpiod_line_set_value(in2, 1);
    lseek(fd, 0, SEEK_SET);
    dprintf(fd, "%d", pwm);
  } else
  {
    // dozadu
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t\tdozadu " << dir << ", " << -pwm;
    gpiod_line_set_value(in1, 1);
    gpiod_line_set_value(in2, 0);
    lseek(fd, 0, SEEK_SET);
    dprintf(fd, "%d", -pwm);
  }
}

/* ***** SPOLOCNE ************************************************************ */
hardware_interface::CallbackReturn DiffDriveAlphabot2::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(get_logger(), "inicializacia..");
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  max_rychlost_kolesa = stod(info_.hardware_parameters["max_rychlost_kolesa"]);
  RCLCPP_INFO(get_logger(), "  maximalna rychlost kolesa = %.1f", max_rychlost_kolesa);
  lave_koleso_nazov = info_.hardware_parameters["lave_koleso_nazov"];
  RCLCPP_INFO(get_logger(), "  lave koleso = %s", lave_koleso_nazov.c_str());
  prave_koleso_nazov = info_.hardware_parameters["prave_koleso_nazov"];
  RCLCPP_INFO(get_logger(), "  prave koleso = %s", prave_koleso_nazov.c_str());

  // kontroly
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // ma prave 1 prikazove rozhranie typu VELOCITY
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(get_logger(), "Joint ’%s’ ma nespravny pocet rozhrani!", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint ’%s’ ma zle prik.rozhr.: %s; ocakava sa ’%s’.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // ma presne 2 stavove rozhrania typu POSITION a VELOCITY
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(get_logger(), "Joint ’%s’ ma %zu stav.rozhrania (namiesto 2).", joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(get_logger(), "Joint ’%s’ ma zle 1. stav.rozhr.: ’%s’", joint.name.c_str(), joint.state_interfaces[0].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint ’%s’ ma zle 2. stav.rozhr.: ’%s’", joint.name.c_str(), joint.state_interfaces[1].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(get_logger(), "nainicializovane");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAlphabot2::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "konfiguracia...");

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "nakonfigurovane");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAlphabot2::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "aktivacia...");

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  if (hw_init() != 0) return CallbackReturn::ERROR;

  RCLCPP_INFO(get_logger(), "aktivovane");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAlphabot2::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "deaktivacia...");
  hw_release();
  RCLCPP_INFO(get_logger(), "deaktivovane");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveAlphabot2::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::stringstream ss;
  ss << "Reading states:" << std::fixed << std::setprecision(2);
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      // Simulate DiffBot wheels's movement as a first-order system
      // Update the joint status: this is a revolute joint without any limit.
      // Simply integrates
      auto velo = get_command(descr.get_prefix_name() + "/" + hardware_interface::HW_IF_VELOCITY);
      set_state(name, get_state(name) + period.seconds() * velo);
      ss << std::endl << "\t position " << get_state(name) << " for '" << name << "'!";
    }
    if (descr.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      auto velo = get_command(descr.get_prefix_name() + "/" + hardware_interface::HW_IF_VELOCITY);
      set_state(name, velo);
      ss << std::endl << "\t velocity " << velo << " for '" << name << "'!";
    }
  }
  //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_alphabot2 ::DiffDriveAlphabot2::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";
  int do_log = 0;
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    // Simulate sending commands to the hardware
    set_state(name, get_command(name));

    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << "command " << get_command(name) << " for '" << name << "'!";
    double velocity_rad_s = get_command(name);  // from ROS2 controller, in rad/s
    if (velocity_rad_s != 0) do_log = 1;

    // calculate pwm
    double max_velocity_rad_s = max_rychlost_kolesa;
    int max_pwm = PERIOD_NS;
    double clamped = std::max(-max_velocity_rad_s, std::min(velocity_rad_s, max_velocity_rad_s));
    double pwm_d = (clamped / max_velocity_rad_s) * max_pwm;
    int pwm = static_cast<int>(std::round(pwm_d));

    if (name == lave_koleso_nazov + "/velocity") {
      hw_write('L', pwm, ss);
    }
    if (name == prave_koleso_nazov + "/velocity") {
      hw_write('P', pwm, ss);
    }

  }
  if (do_log) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_alphabot2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_alphabot2::DiffDriveAlphabot2, hardware_interface::SystemInterface)

