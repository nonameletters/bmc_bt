#adapter_khz 1000
#reset_config none
#transport select jtag

#jtag newtap bj sata -irlen 8  -expected-id 0xa21C64cd -irmask 3 -disable
#jtag newtap bj ddr  -irlen 8  -expected-id 0x00002863 -irmask 3 -disable
#jtag newtap bj apb  -irlen 5  -expected-id 0x00001863 -irmask 3
#jtag newtap bj pci  -irlen 8  -expected-id 0x061c74cd -irmask 3 -disable
#jtag newtap bj xgbe -irlen 8  -expected-id 0x061c74cd -irmask 3 -disable
#jtag newtap bj top  -irlen 16 -expected-id 0x00000863 -irmask 3

jtag_ntrst_assert_width 10
jtag_ntrst_assert_width 100

set pmu.base	0x1f04d000
set gpio	0x1f044000

set pmu_core_debug	[expr ${pmu.base} + 0x118]

array set japc {
 SCAN_ITR 4 SCAN_DTR 5
 EXTEST 0 SCAN_N 2 INTEST 0xC IDCODE 0x1 BYPASS 0XFFFF CRSEL 0x31
 APB_IDCODE 0x1E
 TT_SHORT 0xA001 TT_NORMAL 0xA000 TT_DFT_RESET 0xC000 TT_BIST_RUN 0xB000 TT_BIST_RESULT 0xC100 TT_BIST_RESULT_LENGTH 53
}

proc japb_get_reg {addr} {
  global japc
  irscan bj.apb $japc(SCAN_N)
  drscan bj.apb 5 $japc(SCAN_ITR)
  irscan bj.apb $japc(EXTEST)
  drscan bj.apb 32 $addr 2 0
  set retry 50
  while {[incr retry -1]} {
    irscan bj.apb $japc(INTEST)
    set r [drscan bj.apb 32 0 2 0]
    if { [lindex $r 1] == "00" } { return "0x[lindex $r 0]" }
  }
  error [format "Timeout reading register 0x%X" $addr]
}

proc japb_set_reg {addr value} {
  global japc
  irscan bj.apb $japc(SCAN_N)
  drscan bj.apb 5 $japc(SCAN_DTR)
  irscan bj.apb $japc(EXTEST)
  drscan bj.apb 32 $value
  irscan bj.apb $japc(SCAN_N)
  drscan bj.apb 5 $japc(SCAN_ITR)

  irscan bj.apb $japc(EXTEST)
  drscan bj.apb 34 [expr $addr | 0x100000000]
  irscan bj.apb $japc(INTEST)
  drscan bj.apb 34 0
  runtest 8
  return
}

proc bj_dft_reset {{val 0x1F}} {
  global japc
  irscan bj.top [expr $japc(TT_DFT_RESET) | ($val & 0x1f)]
  runtest 4
  return
}

proc bj_bist_start {val} {
  global japc
  irscan bj.top [expr $japc(TT_BIST_RUN) | ($val & 0xfff)]
  runtest 2
  return
}

proc bj_bist_result {} {
  global japc
  irscan bj.top $japc(TT_BIST_RESULT)
  # 4 done 1 TR_fail 16 P0_fail 16 P1_fail 16 L2_fail
  drscan bj.top 4 0 1 0 16 0 16 0 16 0
}

proc bj_bist_run {{testsel 0xF00}} {
  runtest 64
  japb_get_reg [expr ${::pmu.base} + 0x28] ;# ensure working
  japb_set_reg [expr ${::pmu.base} + 0x28] 0x81 ;# core1 oper
  japb_set_reg [expr ${::pmu.base} + 0x118] 1 ;# reset, self-clear
  runtest 64
  bj_dft_reset 0x1F
  runtest 64
  bj_bist_start $testsel
  set retry 2000
  while {[incr retry -1]} {
    runtest 256
    set rx [bj_bist_result]
    set done "0x[lindex $rx 0]"
    if { ($done & 0xF) == ($testsel >> 8) } {
      bj_dft_reset 0x00
      return $rx
    }
  }
  error [format "Bist test timeout: command %X result %X" $testsel $rx]
}

proc jtt_long {} {
  global japc
  irscan bj.top $japc(TT_NORMAL)
  runtest 2
}

proc ddr_get_reg {addr} {
  global japc
  irscan bj.ddr $japc(CRSEL)
  #drscan bj.ddr 43 [expr ($addr << 2) | 0x1]
  drscan bj.ddr 2 01 10 $addr 32 0
  runtest 15 ;# 8+DWC_CFG_OUT_PIPE
  return "0x[lindex [drscan bj.ddr 2 01 10 $addr 32 0] 2]"
}

proc ddr_set_reg {addr value} {
  global japc
  irscan bj.ddr $japc(CRSEL)
  drscan bj.ddr 2 01 10 $addr 32 $value
  runtest 5
}

proc bj_soft_reset {} { japb_set_reg $::pmu_core_debug 0x1 }


proc jtt_long_chain {} {
  jtag configure bj.sata -event tap-enable ""
  jtag configure bj.ddr  -event tap-enable ""
  jtag configure bj.pci  -event tap-enable ""
  jtag configure bj.xgbe -event tap-enable ""

  jtag tapenable bj.xgbe
  jtag tapenable bj.pci
  jtag tapenable bj.ddr
  jtag tapenable bj.sata
  jtt_long
  jtag arp_init
}

