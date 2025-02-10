# Ethernet issues

NOTE: This was Linux 6.5 in August 2023.

## Entry in GMAC

DTSI

```
gmac0: ethernet@16030000 {
        compatible = "starfive,jh7110-dwmac", "snps,dwmac-5.20";
        reg = <0x0 0x16030000 0x0 0x10000>;
        clocks = <&aoncrg JH7110_AONCLK_GMAC0_AXI>,
                 <&aoncrg JH7110_AONCLK_GMAC0_AHB>,
                 <&syscrg JH7110_SYSCLK_GMAC0_PTP>,
                 <&aoncrg JH7110_AONCLK_GMAC0_TX_INV>,
                 <&syscrg JH7110_SYSCLK_GMAC0_GTXC>;
        clock-names = "stmmaceth", "pclk", "ptp_ref", "tx", "gtx";
        resets = <&aoncrg JH7110_AONRST_GMAC0_AXI>,
                 <&aoncrg JH7110_AONRST_GMAC0_AHB>;
        reset-names = "stmmaceth", "ahb";
        interrupts = <7>, <6>, <5>;
        interrupt-names = "macirq", "eth_wake_irq", "eth_lpi";
        rx-fifo-depth = <2048>;
        tx-fifo-depth = <2048>;
        snps,multicast-filter-bins = <64>;
        snps,perfect-filter-entries = <256>;
        snps,fixed-burst;
        snps,no-pbl-x8;
        snps,force_thresh_dma_mode;
        snps,axi-config = <&stmmac_axi_setup>;
        snps,tso;
        snps,en-tx-lpi-clockgating;
        snps,txpbl = <16>;
        snps,rxpbl = <16>;
        starfive,syscon = <&aon_syscon 0xc 0x12>;
        status = "disabled";
};
```

```
&gmac0 {
        starfive,tx-use-rgmii-clk;
        assigned-clocks = <&aoncrg JH7110_AONCLK_GMAC0_TX>;
        assigned-clock-parents = <&aoncrg JH7110_AONCLK_GMAC0_RMII_RTX>;
        mac-address = [ 46 43 4B 45 46 49 ];
        phy-supply = <&vcc_phy>; // does the trick...
};
```

## How / where is `phy-supply` used?!

```
... = devm_regulator_get(dev, "phy");
```

See `drivers/net/ethernet/stmicro/stmmac/dwmac-rk.c` for an example.

Also documented for some other platforms, e.g.,
`Documentation/devicetree/bindings/net/allwinner,sun4i-a10-mdio.yaml`.

`drivers/net/ethernet/stmicro/stmmac/dwmac-starfive.c` doesn't use it...?

Any value for `phy-supply` appears to work though - even I2C pins phandle?!

## What else?

function `devm_stmmac_probe_config_dt` in
`drivers/net/ethernet/stmicro/stmmac/stmmac_platform.c`

calls into `stmmac_probe_config_dt` (parse device-tree driver parameters)

- gets MAC address
- gets PHY mode

And:

```
/* To Configure PHY by using all device-tree supported properties */
rc = stmmac_dt_phy(plat, np, &pdev->dev);
```

## Conclusion and remarks

I have no idea, really. Adding a regulator dependency deferred probing to late
in the kernel boot process, so when ethernet worked at some point, it was
likely just a side effect. Note that the issue did not occur with the stock
firmware, so it might have brought up some power supplies, clocks etc that we
are not aware of. I am quite certain that it is in U-Boot SPL, because booting
U-Boot via oreboot and then Linux still has issues. Note also that ethernet in
U-Boot itself works when run from oreboot, at least on the Milk-V Mars CM.
