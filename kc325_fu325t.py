#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys

from migen import *

# Closest existing board, making a litex_boards template from my contraint file
from litex_boards.platforms import sitlinv_stlv7325_v2

from litex.build.generic_platform import *
from litex.build.xilinx import VivadoProgrammer

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteeth.phy import LiteEthPHY

from litescope import LiteScopeAnalyzer

from usb3_pipe import K7USB3SerDes, USB3PIPE
from usb3_core.core import USB3Core

# USB3 IOs -----------------------------------------------------------------------------------------

# Pin Map:
# pcie_tx0p		3.3V	A4	PCIe Gen 2.0 x4
# pcie_tx1p		3.3V	B2	
# pcie_tx2p		3.3V	D2	
# pcie_tx3p		3.3V	F2	
# pcie_tx0n		3.3V	A3	
# pcie_tx1n		3.3V	B1	
# pcie_tx2n		3.3V	D1	
# pcie_tx3n		3.3V	F1	
# pcie_rx0p		3.3V	B6	
# pcie_rx1p		3.3V	C4	
# pcie_rx2p		3.3V	E4	
# pcie_rx3p		3.3V	G4	
# pcie_rx0n		3.3V	B5	
# pcie_rx1n		3.3V	C3	
# pcie_rx2n		3.3V	E3	
# pcie_rx3n		3.3V	G3	
# pcie_clkp		3.3V	H6	
# pcie_clkn		3.3V	H5	
# pcie_rst		3.3V	E17
# Pin Map:
# sfp_a_txp	output		H2	
# sfp_a_txn	output		H1	
# sfp_a_rxp	input		J4	
# sfp_a_rxn	input		J3	
# sfp_a_sda	inout		B21	
# sfp_a_scl	output		C21
# Pin Map:
# sfp_b_txp	output		K2	
# sfp_b_txn	output		K1	
# sfp_b_rxp	input		L4	
# sfp_b_rxn	input		L3	
# sfp_b_sda	inout		D21	
# sfp_b_scl	output		D22	
_io = [
        ("clk100", 0,
            Subsignal("p", Pins("F17"), IOStandard("LVCMOS33"))
        ),
        ("clk156", 0, # TODO verify / test (in docs)
            Subsignal("p", Pins("D6"), IOStandard("LVDS")),
            Subsignal("n", Pins("D5"), IOStandard("LVDS")),
        ),
        ("clk125", 0, # TODO verify / test (in docs)
            Subsignal("p", Pins("F6"), IOStandard("LVDS")),
            Subsignal("n", Pins("F5"), IOStandard("LVDS")),
        )
]
_usb3_io = [
    # PCIe / Through PCIsh-to-USB3 breakout board.
    # Note: Check pin out
    # I realistically would prefer PCIe to be used as my debug interface given the lack of io other than ethernet, but etherbone + litescope do give be debug so maybe not
    ("pcie_rx", 0,
        Subsignal("p", Pins("B6")),
        Subsignal("n", Pins("B5")),
    ),
    ("pcie_tx", 0,
        Subsignal("p", Pins("A4")),
        Subsignal("n", Pins("A3")),
    ),

    # SMA
    ("sma_tx", 0,
        Subsignal("p", Pins("K2")),
        Subsignal("n", Pins("K1"))
    ),
    ("sma_rx", 0,
        Subsignal("p", Pins("K6")),
        Subsignal("n", Pins("K5"))
    ),

    # SFP A / Through XillyUSB's SFP2USB.
    # Note Pins not reversed, may need to be for XillyUSB
    ("sfp_tx", 0,
        Subsignal("p", Pins("H2")),
        Subsignal("n", Pins("H1")),
    ),
    ("sfp_rx", 0,
        Subsignal("p", Pins("J4")),
        Subsignal("n", Pins("J3")),
    ),
    '''
    # SFP B / Through XillyUSB's SFP2USB.
    # Note Pins not reversed, may need to be for XillyUSB
    ("sfp_tx", 0,
        Subsignal("p", Pins("K2")),
        Subsignal("n", Pins("K1")),
    ),
    ("sfp_rx", 0,
        Subsignal("p", Pins("L4")),
        Subsignal("n", Pins("L3")),
    ),
    '''
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys    = ClockDomain()
        self.clock_domains.cd_oob    = ClockDomain()
        self.clock_domains.cd_clk125 = ClockDomain()

        # # #

        self.submodules.pll = pll = S7PLL(speedgrade=-2)
        pll.register_clkin(platform.request("clk100"), 100e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq) #?
        pll.create_clkout(self.cd_oob,    sys_clk_freq/4) #?
        pll.register_clkin(platform.request("clk125"), 125e6)
        #pll.create_clkout(self.cd_clk125, 125e6)

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(SoCMini):
    def __init__(self, platform, connector="sfp", with_etherbone=True, with_analyzer=True):
        sys_clk_freq = int(100e6)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident="USB3SoC", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone()

        # Etherbone --------------------------------------------------------------------------------
        if with_etherbone:
            self.submodules.eth_phy = LiteEthPHY(
                clock_pads = platform.request("eth_clocks"),
                pads       = platform.request("eth"),
                clk_freq   = sys_clk_freq)
            self.add_etherbone(phy=self.eth_phy, ip_address="192.168.1.50")

        # USB3 SerDes ------------------------------------------------------------------------------
        usb3_serdes = K7USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = ClockSignal("clk125"),
            refclk_freq  = 125e6,
            tx_pads      = platform.request(connector + "_tx"),
            rx_pads      = platform.request(connector + "_rx"))
        self.submodules += usb3_serdes
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-52]")

        # USB3 PIPE --------------------------------------------------------------------------------
        usb3_pipe = USB3PIPE(serdes=usb3_serdes, sys_clk_freq=sys_clk_freq)
        self.submodules.usb3_pipe = usb3_pipe
        self.comb += usb3_pipe.reset.eq(platform.request("cpu_reset"))

        # USB3 Core --------------------------------------------------------------------------------
        usb3_core = USB3Core(platform)
        self.submodules.usb3_core = usb3_core
        self.comb += [
            usb3_pipe.source.connect(usb3_core.sink),
            usb3_core.source.connect(usb3_pipe.sink),
            usb3_core.reset.eq(~usb3_pipe.ready),
        ]

        # Leds -------------------------------------------------------------------------------------
        self.comb += platform.request("user_led", 0).eq(usb3_serdes.ready)
        self.comb += platform.request("user_led", 1).eq(usb3_pipe.ready)

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            analyzer_signals = [
                # LFPS
                usb3_serdes.tx_idle,
                usb3_serdes.rx_idle,
                usb3_serdes.tx_pattern,
                usb3_serdes.rx_polarity,
                usb3_pipe.lfps.rx_polling,
                usb3_pipe.lfps.tx_polling,

                # Training Sequence
                usb3_pipe.ts.tx_enable,
                usb3_pipe.ts.rx_ts1,
                usb3_pipe.ts.rx_ts2,
                usb3_pipe.ts.tx_enable,
                usb3_pipe.ts.tx_tseq,
                usb3_pipe.ts.tx_ts1,
                usb3_pipe.ts.tx_ts2,
                usb3_pipe.ts.tx_done,

                # LTSSM
                usb3_pipe.ltssm.polling.fsm,
                usb3_pipe.ready,

                # Endpoints
                usb3_serdes.rx_datapath.skip_remover.skip,
                usb3_serdes.source,
                usb3_serdes.sink,
                usb3_pipe.source,
                usb3_pipe.sink,
            ]
            self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 4096, csr_csv="analyzer.csv")

# Build --------------------------------------------------------------------------------------------

import argparse

def main():
    with open("README.md") as f:
        description = [str(f.readline()) for i in range(7)]
    parser = argparse.ArgumentParser(description="".join(description[1:]), formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--build", action="store_true", help="Build bitstream.")
    parser.add_argument("--load",  action="store_true", help="Load bitstream.")
    args = parser.parse_args()

    if not args.build and not args.load:
        parser.print_help()

    os.makedirs("build/xilinx_kc7325/gateware", exist_ok=True)
    os.system("cd usb3_core/daisho && make && ./usb_descrip_gen")
    os.system("cp usb3_core/daisho/usb3/*.init build/xilinx_kc7325/gateware/")
    platform = sitlinv_stlv7325_v2.Platform()
    platform.add_extension(_usb3_io)
    soc     = USB3SoC(platform)
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
