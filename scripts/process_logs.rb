#! /usr/bin/env ruby
require 'eslam'

if ARGV.size < 1 then 
    puts "usage: process_logs.rb log_dir [environment_path]"
    exit
end

eslam = Eslam.new( ARGV[0], ARGV[1] )
eslam.run

