#include <memory>
#include <string>
#include "pluginlib/class_loader.hpp"
#include "hardware_interface/system_interface.hpp"
#include "gtest/gtest.h"

TEST(TestLoadCoppeliaSimHardwareInterface, LoadPlugin)
{
  // Crea un ClassLoader per i plugin che implementano hardware_interface::SystemInterface
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader("hardware_interface", "hardware_interface::SystemInterface");

  try
  {
    // Il nome deve corrispondere a quello definito nel file XML, ad es. "coppelia_description/CoppeliaSimHardwareInterface"
    auto hardware_interface_ptr = loader.createUniqueInstance("cr_hw_configuration/CoppeliaSimHardwareInterface");
    ASSERT_NE(hardware_interface_ptr, nullptr) << "Impossibile caricare il plugin.";
  }
  catch (pluginlib::PluginlibException & ex)
  {
    FAIL() << "Errore nel caricamento del plugin: " << ex.what();
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
