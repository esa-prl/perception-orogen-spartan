#!/usr/bin/env ruby

require 'rock/bundle'
require 'readline'

include Orocos

# Initialize orocos
Bundles.initialize
Bundles.transformer.load_conf('/home/marta/rock/bundles/rover/config/exoter_transformations.rb')

# Setup tasks
Orocos::Process.run 'spartan::Task' => 'spartan',
    'spartan::CameraEmulator' => 'cam_emu' do

    # Apply component configurations
    Orocos.conf.load_dir('../config/')

    cam_emu = Orocos.name_service.get 'cam_emu'
    Orocos.conf.apply(cam_emu, ['default'], :override => true)

    spartan = Orocos.name_service.get 'spartan'
    Orocos.conf.apply(spartan, ['default'], :override => true)

    # Connections first
    spartan.img_in_left.connect_to cam_emu.img_out_left
    spartan.img_in_right.connect_to cam_emu.img_out_right

    # Finish transformer setup
    Orocos.transformer.setup(spartan);

    # Configure and start tasks
    cam_emu.configure
    cam_emu.start
    spartan.configure
    spartan.start

    # Logging
    Orocos.log_all_ports
    spartan.log_all_ports

    # Wait for user interrupt
    Readline::readline('Press <ENTER> to quit...');
end
