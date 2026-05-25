# Run the physical wave probe regression in Vivado XSim.
#
# Run in Vivado Tcl Console:
#   source E:/Vivado/Projects/desperate_yi/MVP2_2D_EE_simulation/scripts/run_physical_wave_probe_sim.tcl
#
# Or from a Windows shell:
#   vivado.bat -mode batch -source E:/Vivado/Projects/desperate_yi/MVP2_2D_EE_simulation/scripts/run_physical_wave_probe_sim.tcl
#
# Optional environment variables:
#   VIVADO_PROJECT=E:/Vivado/Projects/desperate_yi/MVP2_2D_EE_simulation/MVP2_2D_EE_simulation.xpr
#   TESTBENCH_FILE=E:/Vivado/Projects/desperate_yi/MVP2_2D_EE_simulation/sim/tb_fdtd_physical_wave_probe.sv

proc getenv_or_default {name default_value} {
    if {[info exists ::env($name)] && $::env($name) ne ""} {
        return $::env($name)
    }
    return $default_value
}

proc add_file_if_missing {fileset_name path_value file_type} {
    set path_value [file normalize $path_value]
    if {![file exists $path_value]} {
        error "Missing source file: $path_value"
    }
    if {[llength [get_files -quiet -of_objects [get_filesets $fileset_name] $path_value]] == 0} {
        add_files -norecurse -fileset $fileset_name $path_value
    }
    set_property file_type $file_type [get_files -quiet $path_value]
}

set default_project "E:/Vivado/Projects/desperate_yi/MVP2_2D_EE_simulation/MVP2_2D_EE_simulation.xpr"
set project_path [file normalize [getenv_or_default VIVADO_PROJECT $default_project]]

if {[llength [get_projects -quiet]] == 0} {
    open_project $project_path
} else {
    set open_project_dir [file normalize [get_property DIRECTORY [current_project]]]
    set open_project_name [get_property NAME [current_project]]
    set open_project_path [file normalize [file join $open_project_dir "${open_project_name}.xpr"]]
    if {![string equal -nocase $open_project_path $project_path]} {
        close_project
        open_project $project_path
    }
}

set proj_dir [get_property DIRECTORY [current_project]]
set local_solver_dir [file join $proj_dir rtl fdtd_solver_import]
set adapter_file [file join $proj_dir rtl fdtd_solver_bd_adapter.v]
set default_testbench [file join $proj_dir sim tb_fdtd_physical_wave_probe.sv]
set testbench_file [file normalize [getenv_or_default TESTBENCH_FILE $default_testbench]]

if {[llength [get_filesets -quiet sim_1]] == 0} {
    create_fileset -simset sim_1
}

add_file_if_missing sources_1 $adapter_file Verilog
foreach src_name {fdtd_solver.sv fdtd_engine.sv Ey.sv Ex.sv Bz.sv} {
    add_file_if_missing sources_1 [file join $local_solver_dir $src_name] SystemVerilog
}
add_file_if_missing sim_1 $testbench_file SystemVerilog

set_property top tb_fdtd_physical_wave_probe [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
set_property -name {xsim.elaborate.debug_level} -value {typical} -objects [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {all} -objects [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

catch {close_sim}

launch_simulation -simset sim_1 -mode behavioral
close_sim

puts "INFO: Vivado XSim physical wave probe regression completed."
puts "INFO: Testbench: $testbench_file"
