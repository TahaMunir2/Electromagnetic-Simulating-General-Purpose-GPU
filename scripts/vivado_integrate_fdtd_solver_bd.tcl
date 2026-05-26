# Integrate Taha's 2D fdtd_solver into the existing MVP2 Vivado block design.
#
# Run in Vivado Tcl Console:
#   source E:/Vivado/Projects/desperate_yi/MVP2_2D_EE_simulation/scripts/integrate_fdtd_solver_bd.tcl
#
# Optional environment variables:
#   VIVADO_PROJECT=E:/Vivado/Projects/desperate_yi/MVP2_2D_EE_simulation/MVP2_2D_EE_simulation.xpr
#   REPO_ROOT=E:/path/to/Electromagnetic-Simulating-General-Purpose-GPU
#   RUN_SYNTH=1
#   RUN_IMPL=1
#   VIVADO_JOBS=4
#
# This script keeps one physical blk_mem_gen per field:
#   ey_bram, ex_bram, bz_bram, s_mag_bram
#
# The fdtd_solver_bd_adapter maps the solver's logical ports onto each true
# dual-port BRAM:
#   - port A: main read
#   - port B: write during that field's update phase, otherwise adjacent read
#
# After solver_done, the adapter scans Ex/Ey/Bz and writes a render magnitude
# into s_mag_bram:
#   mag_mode=0: |E| ~= max(abs(Ex), abs(Ey)) + min(abs(Ex), abs(Ey))/2
#   mag_mode=1: |S| ~= (abs(Bz) * |E|) >> 13

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
    if {[llength [get_files -quiet $path_value]] == 0} {
        add_files -norecurse -fileset $fileset_name $path_value
    }
    set_property file_type $file_type [get_files $path_value]
}

proc remove_stale_source_if_present {path_value} {
    set path_value [file normalize $path_value]
    set file_obj [get_files -quiet $path_value]
    if {[llength $file_obj] != 0} {
        remove_files $file_obj
    }
    if {[file exists $path_value]} {
        file delete -force $path_value
    }
}

proc disconnect_pin_if_connected {pin_name} {
    set pin [get_bd_pins -quiet $pin_name]
    if {[llength $pin] == 0} {
        return
    }
    set net [get_bd_nets -quiet -of_objects $pin]
    if {[llength $net] != 0} {
        catch {disconnect_bd_net $net $pin}
    }
}

proc ensure_port {name dir args} {
    set port [get_bd_ports -quiet $name]
    if {[llength $port] == 0} {
        set port [eval create_bd_port -dir $dir $args $name]
    }
    return $port
}

proc ensure_const {name width value} {
    set cell [get_bd_cells -quiet $name]
    if {[llength $cell] == 0} {
        set cell [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 $name]
    }
    set_property -dict [list CONFIG.CONST_WIDTH $width CONFIG.CONST_VAL $value] $cell
    return $cell
}

proc disconnect_bram_data_pins {bram} {
    foreach pin {addra addrb dina dinb douta doutb ena enb wea web clka clkb} {
        disconnect_pin_if_connected "$bram/$pin"
    }
}

proc wire_solver_bram {bram prefix} {
    set adapter fdtd_solver_bd_adapter_0

    disconnect_bram_data_pins $bram

    connect_bd_net [get_bd_ports clk] [get_bd_pins $bram/clka]
    connect_bd_net [get_bd_ports clk] [get_bd_pins $bram/clkb]

    connect_bd_net [get_bd_pins $adapter/${prefix}_addra] [get_bd_pins $bram/addra]
    connect_bd_net [get_bd_pins $adapter/${prefix}_ena]   [get_bd_pins $bram/ena]
    connect_bd_net [get_bd_pins $adapter/${prefix}_wea]   [get_bd_pins $bram/wea]
    connect_bd_net [get_bd_pins $adapter/${prefix}_dina]  [get_bd_pins $bram/dina]
    connect_bd_net [get_bd_pins $bram/douta]              [get_bd_pins $adapter/${prefix}_douta]

    connect_bd_net [get_bd_pins $adapter/${prefix}_addrb] [get_bd_pins $bram/addrb]
    connect_bd_net [get_bd_pins $adapter/${prefix}_enb]   [get_bd_pins $bram/enb]
    connect_bd_net [get_bd_pins $adapter/${prefix}_web]   [get_bd_pins $bram/web]
    connect_bd_net [get_bd_pins $adapter/${prefix}_dinb]  [get_bd_pins $bram/dinb]

    if {[llength [get_bd_pins -quiet $adapter/${prefix}_doutb]] != 0} {
        connect_bd_net [get_bd_pins $bram/doutb] [get_bd_pins $adapter/${prefix}_doutb]
    }
}

proc tie_off_bram {bram} {
    disconnect_bram_data_pins $bram

    connect_bd_net [get_bd_ports clk] [get_bd_pins $bram/clka]
    connect_bd_net [get_bd_ports clk] [get_bd_pins $bram/clkb]
    connect_bd_net [get_bd_pins const_1/dout]    [get_bd_pins $bram/ena]
    connect_bd_net [get_bd_pins const_1/dout]    [get_bd_pins $bram/enb]
    connect_bd_net [get_bd_pins const_0_1/dout]  [get_bd_pins $bram/wea]
    connect_bd_net [get_bd_pins const_0_1/dout]  [get_bd_pins $bram/web]
    connect_bd_net [get_bd_pins const_0_16/dout] [get_bd_pins $bram/addra]
    connect_bd_net [get_bd_pins const_0_16/dout] [get_bd_pins $bram/addrb]
    connect_bd_net [get_bd_pins const_0_16/dout] [get_bd_pins $bram/dina]
    connect_bd_net [get_bd_pins const_0_16/dout] [get_bd_pins $bram/dinb]
}

proc refresh_module_reference_for_pin {cell_name module_name required_pin} {
    update_compile_order -fileset sources_1
    if {[llength [get_bd_pins -quiet "$cell_name/$required_pin"]] != 0} {
        return
    }

    set refresh_sets [list \
        [get_bd_cells -quiet $cell_name] \
        [get_ips -quiet "*${cell_name}*"] \
        [get_ips -quiet "*${module_name}*"] \
        [list $module_name] \
    ]

    foreach refs $refresh_sets {
        if {[llength $refs] == 0} {
            continue
        }
        puts "INFO: Refreshing module reference for $cell_name using: $refs"
        set status [catch {update_module_reference $refs} result]
        if {$status != 0} {
            puts "INFO: update_module_reference did not accept '$refs': $result"
        } else {
            puts "INFO: update_module_reference result: $result"
        }
        if {[llength [get_bd_pins -quiet "$cell_name/$required_pin"]] != 0} {
            return
        }
    }

    set available_pins [lsort [get_property NAME [get_bd_pins -quiet "$cell_name/*"]]]
    error "Module reference '$cell_name' did not expose required pin '$required_pin'. Available pins: $available_pins. The RTL file has likely changed but Vivado is still using a stale module-reference interface."
}

set default_project "E:/Vivado/Projects/desperate_yi/MVP2_2D_EE_simulation/MVP2_2D_EE_simulation.xpr"
set project_path [file normalize [getenv_or_default VIVADO_PROJECT $default_project]]
set jobs [getenv_or_default VIVADO_JOBS "4"]
set run_synth [getenv_or_default RUN_SYNTH "0"]
set run_impl [getenv_or_default RUN_IMPL "0"]

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
set bd_file [file join $proj_dir "MVP2_2D_EE_simulation.srcs" "sources_1" "bd" "mvp2_ftdt_bd" "mvp2_ftdt_bd.bd"]
set local_solver_dir [file join $proj_dir rtl fdtd_solver_import]
set adapter_file [file join $proj_dir rtl fdtd_solver_bd_adapter.v]
set repo_root [getenv_or_default REPO_ROOT ""]

if {$repo_root ne ""} {
    set repo_root [file normalize $repo_root]
    set repo_adapter [file join $repo_root vivado fdtd_solver_bd_adapter.v]
    if {![file exists $repo_adapter]} {
        error "REPO_ROOT was set, but adapter file is missing: $repo_adapter"
    }
    file mkdir [file dirname $adapter_file]
    file copy -force $repo_adapter $adapter_file

    foreach src_name {fdtd_solver.sv fdtd_engine.sv Ey.sv Ex.sv Bz.sv} {
        set src_file [file join $repo_root src hdl $src_name]
        if {![file exists $src_file]} {
            error "REPO_ROOT was set, but source file is missing: $src_file"
        }
        file mkdir $local_solver_dir
        file copy -force $src_file [file join $local_solver_dir $src_name]
    }
}

set solver_files [list \
    [file join $local_solver_dir fdtd_solver.sv] \
    [file join $local_solver_dir fdtd_engine.sv] \
    [file join $local_solver_dir Ey.sv] \
    [file join $local_solver_dir Ex.sv] \
    [file join $local_solver_dir Bz.sv] \
]

foreach stale_file [list \
    [file join $proj_dir rtl bram_test_adapter.v] \
    [file join $proj_dir rtl cordic_source_adapter.sv] \
    [file join $proj_dir "MVP2_2D_EE_simulation.srcs" "sources_1" "new" "cordic_source_adapter.sv"] \
] {
    remove_stale_source_if_present $stale_file
}

add_file_if_missing sources_1 $adapter_file Verilog
foreach src_file $solver_files {
    add_file_if_missing sources_1 $src_file SystemVerilog
}
update_compile_order -fileset sources_1

catch {close_design}
open_bd_design $bd_file

foreach obj_name {
    bram_test_start bram_busy bram_done bram_pass
    solver_enable solver_done source_latched solver_checksum
    e_mag_busy e_mag_done mag_mode mag_busy mag_done
} {
    catch {delete_bd_objs [get_bd_ports -quiet $obj_name]}
}

catch {delete_bd_objs [get_bd_cells -quiet bram_test_adapter_0]}
catch {delete_bd_objs [get_bd_cells -quiet fdtd_solver_bd_adapter_0]}
catch {delete_bd_objs [get_bd_cells -quiet ey_adj_bram]}
catch {delete_bd_objs [get_bd_cells -quiet bz_adj_bram]}

ensure_const const_1 1 1
ensure_const const_0_1 1 0
ensure_const const_0_16 16 0

set adapter_cell [create_bd_cell -type module -reference fdtd_solver_bd_adapter fdtd_solver_bd_adapter_0]
refresh_module_reference_for_pin fdtd_solver_bd_adapter_0 fdtd_solver_bd_adapter mag_done

ensure_port rst I -type rst
set_property CONFIG.POLARITY ACTIVE_HIGH [get_bd_ports rst]
connect_bd_net [get_bd_ports clk] [get_bd_pins fdtd_solver_bd_adapter_0/clk]
connect_bd_net [get_bd_ports rst] [get_bd_pins fdtd_solver_bd_adapter_0/rst]

ensure_port solver_enable I
ensure_port mag_mode I
ensure_port solver_done O
ensure_port source_latched O
ensure_port solver_checksum O -from 31 -to 0
ensure_port mag_busy O
ensure_port mag_done O
connect_bd_net [get_bd_ports solver_enable] [get_bd_pins fdtd_solver_bd_adapter_0/solver_enable]
connect_bd_net [get_bd_ports mag_mode] [get_bd_pins fdtd_solver_bd_adapter_0/mag_mode]
connect_bd_net [get_bd_ports solver_done] [get_bd_pins fdtd_solver_bd_adapter_0/solver_done]
connect_bd_net [get_bd_ports source_latched] [get_bd_pins fdtd_solver_bd_adapter_0/source_latched]
connect_bd_net [get_bd_ports solver_checksum] [get_bd_pins fdtd_solver_bd_adapter_0/solver_checksum]
connect_bd_net [get_bd_ports mag_busy] [get_bd_pins fdtd_solver_bd_adapter_0/mag_busy]
connect_bd_net [get_bd_ports mag_done] [get_bd_pins fdtd_solver_bd_adapter_0/mag_done]

connect_bd_net [get_bd_pins cordic_source_adapter_0/source_q313] \
               [get_bd_pins fdtd_solver_bd_adapter_0/source_q313]
connect_bd_net [get_bd_pins cordic_source_adapter_0/source_valid] \
               [get_bd_pins fdtd_solver_bd_adapter_0/source_valid]

wire_solver_bram ey_bram ey
wire_solver_bram ex_bram ex
wire_solver_bram bz_bram bz
wire_solver_bram s_mag_bram s_mag

regenerate_bd_layout -routing
validate_bd_design
save_bd_design

generate_target all [get_files $bd_file]
set wrapper_file [make_wrapper -files [get_files $bd_file] -top]
if {[file exists $wrapper_file]} {
    add_files -norecurse -force $wrapper_file
} else {
    set wrapper_file [file join $proj_dir "MVP2_2D_EE_simulation.gen" "sources_1" "bd" "mvp2_ftdt_bd" "hdl" "mvp2_ftdt_bd_wrapper.v"]
    add_files -norecurse -force $wrapper_file
}

set_property top mvp2_ftdt_bd_wrapper [get_filesets sources_1]
update_compile_order -fileset sources_1

set report_dir [file join $proj_dir reports_solver_integrated]
file mkdir $report_dir

set preserve_xdc [file join $proj_dir solver_preserve.xdc]
set preserve_fp [open $preserve_xdc w]
puts $preserve_fp {
# Temporary preservation constraints for MVP2 solver bring-up.
# The checksum output makes solver writes observable; these DONT_TOUCH
# constraints keep the physical BRAM/IP structure present until the renderer
# consumer and final FSM are wired.
set_property DONT_TOUCH true [get_cells -hier -quiet {*mvp2_ftdt_bd_i/ey_bram*}]
set_property DONT_TOUCH true [get_cells -hier -quiet {*mvp2_ftdt_bd_i/ex_bram*}]
set_property DONT_TOUCH true [get_cells -hier -quiet {*mvp2_ftdt_bd_i/bz_bram*}]
set_property DONT_TOUCH true [get_cells -hier -quiet {*mvp2_ftdt_bd_i/s_mag_bram*}]
set_property DONT_TOUCH true [get_cells -hier -quiet {*mvp2_ftdt_bd_i/fdtd_solver_bd_adapter_0*}]
}
close $preserve_fp

if {[llength [get_files -quiet $preserve_xdc]] == 0} {
    add_files -fileset constrs_1 -norecurse $preserve_xdc
}

if {$run_synth eq "1" || $run_impl eq "1"} {
    reset_run synth_1
    launch_runs synth_1 -jobs $jobs
    wait_on_run synth_1
    set synth_status [get_property STATUS [get_runs synth_1]]
    puts "INFO: synth_1 status: $synth_status"
    if {[string first "synth_design Complete" $synth_status] < 0} {
        error "synth_1 did not complete successfully: $synth_status"
    }

    open_run synth_1
    report_utilization -file [file join $report_dir utilization_synth_solver.rpt]
    report_timing_summary -max_paths 10 -file [file join $report_dir timing_synth_solver.rpt]
    report_drc -file [file join $report_dir drc_synth_solver.rpt]
    close_design
}

if {$run_impl eq "1"} {
    reset_run impl_1
    launch_runs impl_1 -to_step route_design -jobs $jobs
    wait_on_run impl_1
    set impl_status [get_property STATUS [get_runs impl_1]]
    puts "INFO: impl_1 status: $impl_status"
    if {[string first "route_design Complete" $impl_status] < 0} {
        error "impl_1 did not route successfully: $impl_status"
    }

    open_run impl_1
    report_utilization -file [file join $report_dir utilization_impl_solver.rpt]
    report_timing_summary -max_paths 10 -file [file join $report_dir timing_impl_solver.rpt]
    report_route_status -file [file join $report_dir route_status_solver.rpt]
    report_drc -file [file join $report_dir drc_impl_solver.rpt]
    close_design
}

puts "INFO: FDTD solver integration script completed."
puts "INFO: BD now uses one physical BRAM per field plus fdtd_solver_bd_adapter_0."
puts "INFO: Port B is muxed between adjacent read and write where needed."
puts "INFO: mag_busy/mag_done indicate the post-solver render magnitude pass."
puts "INFO: mag_mode=0 stores |E| ~= max(abs(Ex),abs(Ey)) + min(abs(Ex),abs(Ey))/2."
puts "INFO: mag_mode=1 stores |S| ~= (abs(Bz) * |E|) >> 13."
puts "INFO: solver_checksum[31:0] is exported to preserve/observe solver activity."
puts "INFO: Temporary preservation constraints written to: $preserve_xdc"
puts "INFO: Optional reports directory: $report_dir"
