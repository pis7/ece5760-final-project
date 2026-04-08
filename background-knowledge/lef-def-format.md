# LEF/DEF Format Reference

This document describes the LEF and DEF file formats as used by the ICCAD 2004
Faraday mixed-size benchmarks in `benchmarks/iccad04/`.

## LEF (Library Exchange Format) — The Library

The LEF file defines the **technology and cell library**: everything the placer
needs to know about the physical building blocks, but nothing about a specific
design.

### Sections

#### 1. Technology Header

Units, manufacturing grid, and metal/via layer stack.

```
VERSION 5.4 ;
UNITS
    DATABASE MICRONS 1000 ;
END UNITS
```

All coordinates in LEF/DEF are in database units. With `DATABASE MICRONS 1000`,
1 database unit = 0.001 microns (1 nm).

#### 2. Layer Definitions

Each metal and via layer with routing rules:

```
LAYER metal1
    TYPE ROUTING ;
    WIDTH 0.160 ;          -- minimum wire width (um)
    SPACING 0.160 ;        -- minimum spacing (um)
    PITCH 0.400 ;          -- routing track pitch (um)
    DIRECTION HORIZONTAL ; -- preferred routing direction
    RESISTANCE RPERSQ 0.07090000 ;
    CAPACITANCE CPERSQDIST 9.7045e-05 ;
END metal1
```

Via/cut layers separate metal layers:
```
LAYER V1
    TYPE CUT ;
    SPACING 0.28 ;
END V1
```

The full stack in these benchmarks is:
PC (poly) → CA (contact) → metal1 → V1 → metal2 → V2 → metal3 → V3 → metal4
→ VL → metal5 → VH → metal6

#### 3. SITE Definitions

The smallest legal placement slot for standard cells:

```
SITE cellsite
    CLASS CORE ;
    SYMMETRY x y ;
    SIZE 0.400 BY 3.600 ;  -- width x height in um
END cellsite
```

Standard cells must be placed at integer multiples of the site width (0.400 um)
along rows, and row height is 3.600 um.

#### 4. MACRO Definitions

Each standard cell or hard macro (e.g., SRAM) in the library:

```
MACRO MAS1
    CLASS CORE ;                  -- standard cell (vs. BLOCK for hard macros)
    SIZE 1.600 BY 3.600 ;        -- cell dimensions (4 sites wide, 1 row tall)
    SYMMETRY x y ;               -- allowed orientations
    SITE cellsite ;              -- placement site type
    PIN Y
        DIRECTION OUTPUT ;
        PORT
        LAYER metal1 ;
        RECT 0.690 0.800 1.240 1.080 ;  -- pin access rectangle (llx lly urx ury)
        END
    END Y
    PIN A
        DIRECTION INPUT ;
        PORT
        LAYER metal1 ;
        RECT 0.850 1.240 1.160 1.730 ;
        END
    END A
    PIN VSS
        DIRECTION INOUT ;
        USE ground ;
        SHAPE ABUTMENT ;         -- power rail shared with adjacent cells
        ...
    END VSS
END MAS1
```

Key fields:
- **CLASS**: `CORE` for standard cells, `BLOCK` for hard macros (SRAMs, etc.)
- **SIZE**: Physical dimensions. Width is always a multiple of site width.
- **PIN**: Each pin has a direction, metal layer, and geometry rectangles
  defining where the router can connect. Power pins (VDD/VSS) use
  `SHAPE ABUTMENT` for shared power rails between abutting cells.

## DEF (Design Exchange Format) — The Design

The DEF file describes a **specific design instance** that references the LEF
library.

### Sections

#### 1. Header

```
DESIGN DSP_CORE ;
UNITS DISTANCE MICRONS 1000 ;
DIEAREA ( -321200 -321200 ) ( 321600 321600 ) ;
```

`DIEAREA` defines the rectangular chip boundary as (lower-left) (upper-right)
in database units. DSP2's die is ~642.8 x 642.8 um.

#### 2. ROW Definitions

Legal placement rows where standard cells can be placed:

```
ROW ROW_177 cellsite -321200 317200 FS DO 1607 BY 1 STEP 400 0 ;
ROW ROW_176 cellsite -321200 313600 N  DO 1607 BY 1 STEP 400 0 ;
```

Fields: `ROW <name> <site> <x_origin> <y_origin> <orient> DO <num_sites> BY 1 STEP <site_width> 0`

- Rows alternate between N (normal) and FS (flipped-south) orientation so
  adjacent rows share VDD or VSS power rails.
- `DO 1607 BY 1 STEP 400 0` means 1607 sites spaced 400 db-units apart.
- Row pitch = 3600 db-units (3.6 um), matching the site height.

#### 3. COMPONENTS

Every cell instance in the design, referencing a LEF MACRO:

```
COMPONENTS 26281 ;
- glog/ckT_CLKI/U1  MAS1  + PLACED ( 141200 -10400 ) N ;
- glog/U13           MAS2  + PLACED ( 140400 -320000 ) N ;
...
END COMPONENTS
```

Fields: `- <instance_name> <macro_name> + PLACED ( <x> <y> ) <orientation> ;`

- **instance_name**: Hierarchical name from the synthesized netlist
- **macro_name**: References a MACRO defined in the LEF
- **PLACED**: Initial placement position (this is what the placer optimizes)
- **orientation**: N, S, E, W, FN, FS, FE, FW

For the placer, this provides the initial cell positions. Cell dimensions come
from the MACRO's SIZE in the LEF.

#### 4. PINS (I/O Pads)

Top-level I/O ports placed on the die boundary:

```
PINS 846 ;
- VSS + NET VSS + DIRECTION INPUT + USE GROUND ;
- CMAinx[13] + NET CMAinx[13] + DIRECTION OUTPUT + USE SIGNAL
  + LAYER metal3 ( -100 0 ) ( 100 900 ) + PLACED ( 321600 -58600 ) W ;
...
END PINS
```

These are **fixed** — the placer does not move them. In the quadratic
formulation, connections to fixed I/O pins contribute to the **c** vector
(right-hand side) rather than the **Q** matrix.

#### 5. NETS

The connectivity (netlist). Each net lists the component pins it connects:

```
NETS 28431 ;
- PMAinx[12] ( PIN PMAinx[12] ) ( U195 Y ) ;
- n88 ( U192 A ) ( U191 Y ) ;
...
END NETS
```

Fields: `- <net_name> ( <comp> <pin> ) ( <comp> <pin> ) ... ;`

- `( PIN <name> )` references a top-level I/O pin from the PINS section
- `( <instance> <pin> )` references a pin on a placed component
- Multi-pin nets (3+ pins) are common and must be decomposed (e.g., clique
  model) for the quadratic wirelength formulation

This is the primary input for building the connectivity matrix **Q**.

## Relationship Between LEF and DEF

```
LEF (library)                    DEF (design)
─────────────                    ────────────
LAYER defs          ◄───────     (referenced implicitly)
SITE cellsite       ◄───────     ROW definitions use site type
MACRO MAS1          ◄───────     COMPONENTS reference macro names
  SIZE 1.6 x 3.6                   instance + position + orient
  PIN A, B, Y                    NETS reference instance.pin
```

The LEF is shared across designs (same technology/library), while each design
has its own DEF.

## ICCAD 2004 Benchmark Summary

| Design | Unique MACROs | Components | I/O Pins | Nets   | Die Size (um)  |
|--------|--------------|------------|----------|--------|----------------|
| DMA    | 93           | 11,734     | 950      | 13,256 | 408 x 408      |
| DSP1   | 295          | 26,301     | 846      | 28,447 | 706 x 706      |
| DSP2   | 293          | 26,281     | 846      | 28,431 | 643 x 643      |
| RISC1  | 147          | 32,622     | 629      | 34,034 | 1,003 x 1,003  |
| RISC2  | 147          | 32,622     | 629      | 34,034 | 960 x 960      |

All benchmarks use IBM 0.13um technology (Artisan libraries) with 6 metal
layers. DSP1/DSP2 are variants of a 16-bit DSP (DSP1 uses only SRAMs, DSP2 uses
SRAMs + register files). RISC1/RISC2 are variants of a 32-bit RISC CPU with the
same distinction. DMA is a DMA controller.
