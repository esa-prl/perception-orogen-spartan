#!/usr/bin/env ruby

require 'rock/bundle'
require 'readline'

include Orocos

# Initialize orocos
Bundles.initialize
Bundles.transformer.load_conf('/home/marta/rock/bundles/rover/config/transforms_scripts_exoter.rb')

# Setup tasks
Orocos::Process.run 'spartan::MappingTask' => 'mapper',
    'spartan::CameraEmulator' => 'cam_emu' do

    # Apply component configurations
    Orocos.conf.load_dir('../config/')

    cam_emu = Orocos.name_service.get 'cam_emu'
    Orocos.conf.apply(cam_emu, ['mapping'], :override => true)

    mapper = Orocos.name_service.get 'mapper'
    Orocos.conf.apply(mapper, ['default'], :override => true)

    # Connections first
    mapper.img_in_left.connect_to cam_emu.img_out_left
    mapper.img_in_right.connect_to cam_emu.img_out_right
    mapper.world2body.connect_to cam_emu.world2body_transform

    # Finish transformer setup
    Orocos.transformer.setup(mapper);

    # Configure and start tasks
    cam_emu.configure
    cam_emu.start
    mapper.configure
    mapper.start

    # Logging
    mapper.log_all_ports
    cam_emu.log_all_ports

    # Wait for user interrupt
    Readline::readline('Press <ENTER> to quit...');
end
