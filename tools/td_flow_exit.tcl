# Wrapper around Anlogic TD's DefaultFlow.tcl that guarantees the process exits.
#
# WHY THIS FILE EXISTS
#   D:/Download/TD/scripts/DefaultFlow.tcl contains no `exit` / `quit`. Once it
#   finishes, td_commands_prompt.exe drops back into its interactive prompt and
#   blocks forever waiting on stdin. The vendor's own run.bat appends a `pause`
#   that is therefore never reached either -- in the GUI workflow a human just
#   closes the window.
#
#   In a headless build this is fatal in a non-obvious way: the driver script's
#   syn stage never returns, so the phy stage is never started, and the only
#   symptom is a build that silently stops half way with no error anywhere.
#
#   Sourcing the flow from here and exiting explicitly makes the process
#   terminate and lets us propagate a meaningful exit code.
#
# USAGE / CONSTRAINTS
#   * DefaultFlow.tcl does `source ./settings.cfg`, i.e. it reads its config
#     from the CURRENT WORKING DIRECTORY. The caller must cd into the run
#     directory (syn_1 / phy_1) first. This wrapper itself may live anywhere.
#   * Pass this file's path to td_commands_prompt.exe using FORWARD slashes.
#     Tcl treats backslashes in the argument as escape characters, so
#     "d:\...\td_flow_exit.tcl" arrives mangled and fails to load.
#   * The flow script location can be overridden with the TD_FLOW_SCRIPT
#     environment variable (used by tools/td_build.ps1).

if {[info exists ::env(TD_FLOW_SCRIPT)] && [string length $::env(TD_FLOW_SCRIPT)] > 0} {
    set _td_flow_script $::env(TD_FLOW_SCRIPT)
} else {
    set _td_flow_script {D:/Download/TD/scripts/DefaultFlow.tcl}
}

# `catch` is required because DefaultFlow.tcl signals a failed step with
# `return -code error`, which would otherwise abort before we could exit(1).
set _td_flow_rc [catch {source $_td_flow_script} _td_flow_err]

if {$_td_flow_rc} {
    puts "TD_FLOW_WRAPPER: flow script raised an error:"
    puts $_td_flow_err
    exit 1
}

exit 0
