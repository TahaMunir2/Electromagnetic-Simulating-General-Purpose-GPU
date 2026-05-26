# Ey/Ex Parallelisation — Implementation Guide

---

## Current structure

Each iteration runs three sequential passes, one cell per clock:

```
Phase 1  counter 0          to GRID-1        Ey update   36,864 cycles
Phase 2  counter GRID       to 2*GRID-1      Ex update   36,864 cycles
Phase 3  counter 2*GRID     to 3*GRID-1      Bz update   36,864 cycles
                                                         ─────────────
                                                         110,592 total
```

The goal is to merge phases 1 and 2 so Ey and Ex are computed in the same
pass, then run Bz after:

```
Phase 1  counter 0          to GRID-1        Ey + Ex update   36,864 cycles
Phase 2  counter GRID       to 2*GRID-1      Bz update        36,864 cycles
                                                               ───────────────
                                                               73,728 total
```

---

## Why the merge works

The Ey and Ex update equations only read from Bz^n (old Bz) and their own
old field values. They never read from each other:

```
Ey[i,j]^new = ca_ey * Ey[i,j]^old + cb_ey * (Bz[i,j]^old - Bz[i-1,j]^old)
Ex[i,j]^new = ca_ex * Ex[i,j]^old - cb_ex * (Bz[i,j]^old - Bz[i,j-1]^old)
```

They write to separate BRAMs. There is no dependency. They can run on the
same counter value simultaneously.

Bz must still wait because it reads the NEW Ey and Ex values:

```
Bz[i,j]^new = ca_bz * Bz[i,j]^old
            + cb_bz * ((Ey[i+1,j]^new - Ey[i,j]^new) - (Ex[i,j+1]^new - Ex[i,j]^new))
```

---

## The one structural problem: `bz_left`

In `fdtd_engine.sv`, `u_ey` and `u_ex` currently share a single `bz_left`
input:

```systemverilog
input  logic signed [FP_WIDTH-1:0] bz_left,   // goes to both u_ey and u_ex
...
ey u_ey ( .bz_left(bz_left), ... );
ex u_ex ( .bz_left(bz_left), ... );
```

In the merged pass they need different values at the same cycle:

```
u_ey needs bz_left = Bz[i-1, j]   (the row above — from bz_adj port)
u_ex needs bz_left = Bz[i, j-1]   (the col to the left — from prev_bz register)
```

A single port cannot supply both. `bz_left` must be split into two.

---

## Change 1 — `fdtd_engine.sv`

Split `bz_left` into `bz_left_ey` and `bz_left_ex` and route each to the
correct submodule.

**Port list — before:**
```systemverilog
input  logic signed [FP_WIDTH-1:0] bz_left,
input  logic signed [FP_WIDTH-1:0] bz_right,
```

**Port list — after:**
```systemverilog
input  logic signed [FP_WIDTH-1:0] bz_left_ey,
input  logic signed [FP_WIDTH-1:0] bz_left_ex,
input  logic signed [FP_WIDTH-1:0] bz_right,
```

**`u_ey` instantiation — before:**
```systemverilog
ey #(.FP_WIDTH(FP_WIDTH)) u_ey (
    .clk(clk),
    .ca(ca_ey),
    .cb(cb_ey),
    .ey_old(ey_old),
    .bz_left(bz_left),
    .bz_right(bz_right),
    .ey_new(ey_new)
);
```

**`u_ey` instantiation — after:**
```systemverilog
ey #(.FP_WIDTH(FP_WIDTH)) u_ey (
    .clk(clk),
    .ca(ca_ey),
    .cb(cb_ey),
    .ey_old(ey_old),
    .bz_left(bz_left_ey),
    .bz_right(bz_right),
    .ey_new(ey_new)
);
```

**`u_ex` instantiation — before:**
```systemverilog
ex #(.FP_WIDTH(FP_WIDTH)) u_ex (
    .clk(clk),
    .ca(ca_ex),
    .cb(cb_ex),
    .ex_old(ex_old),
    .bz_left(bz_left),
    .bz_right(bz_right),
    .ex_new(ex_new)
);
```

**`u_ex` instantiation — after:**
```systemverilog
ex #(.FP_WIDTH(FP_WIDTH)) u_ex (
    .clk(clk),
    .ca(ca_ex),
    .cb(cb_ex),
    .ex_old(ex_old),
    .bz_left(bz_left_ex),
    .bz_right(bz_right),
    .ex_new(ex_new)
);
```

`u_bz` is untouched — it has its own separate inputs and is not involved in
this change.

---

## Change 2 — `fdtd_solver.sv`

There are four distinct areas to update.

---

### 2a. Signal declarations

Rename `engine_bz_left` to two separate signals, one per field component.

**Before:**
```systemverilog
logic signed [DATA_WIDTH-1:0] engine_bz_left;
```

**After:**
```systemverilog
logic signed [DATA_WIDTH-1:0] engine_bz_left_ey;
logic signed [DATA_WIDTH-1:0] engine_bz_left_ex;
```

`THREE_GRID_SIZE` is no longer needed. Remove it:

**Before:**
```systemverilog
localparam logic [2*CELL_WIDTH+1:0] GRID_SIZE       = CELLS*CELLS;
localparam logic [2*CELL_WIDTH+1:0] TWO_GRID_SIZE   = 2*GRID_SIZE;
localparam logic [2*CELL_WIDTH+1:0] THREE_GRID_SIZE = 3*GRID_SIZE;
```

**After:**
```systemverilog
localparam logic [2*CELL_WIDTH+1:0] GRID_SIZE     = CELLS*CELLS;
localparam logic [2*CELL_WIDTH+1:0] TWO_GRID_SIZE = 2*GRID_SIZE;
```

---

### 2b. `fdtd_engine` instantiation

**Before:**
```systemverilog
fdtd_engine #(.FP_WIDTH(DATA_WIDTH)) fdtd_engine (
    ...
    .bz_left(engine_bz_left),
    ...
);
```

**After:**
```systemverilog
fdtd_engine #(.FP_WIDTH(DATA_WIDTH)) fdtd_engine (
    ...
    .bz_left_ey(engine_bz_left_ey),
    .bz_left_ex(engine_bz_left_ex),
    ...
);
```

---

### 2c. `always_comb` — phase address

The three-way split reduces to two phases.

**Before:**
```systemverilog
if (counter < GRID_SIZE) begin
    phase_addr = counter;
end else if (counter < TWO_GRID_SIZE) begin
    phase_addr = counter - GRID_SIZE;
end else begin
    phase_addr = counter - TWO_GRID_SIZE;
end
```

**After:**
```systemverilog
if (counter < GRID_SIZE) begin
    phase_addr = counter;
end else begin
    phase_addr = counter - GRID_SIZE;
end
```

---

### 2d. `always_comb` — read address and engine input logic

This is the most substantial change. The three-phase `if/else if/else if`
becomes two phases.

**Before:**
```systemverilog
engine_ey_left  = prev_ey;
engine_ey_right = ey_rd_dout;
engine_bz_left  = prev_bz;

if (counter < GRID_SIZE) begin
    ey_rd_addr = cell_addr;
    bz_rd_addr = cell_addr;
    if (row != 0) begin
        bz_adj_rd_addr = cell_addr - CELLS;
        engine_bz_left = bz_adj_dout;
    end
end else if (counter < TWO_GRID_SIZE) begin
    ex_rd_addr = cell_addr;
    bz_rd_addr = cell_addr;
end else if (counter < THREE_GRID_SIZE) begin
    bz_rd_addr = cell_addr;
    ey_rd_addr = cell_addr;
    if (column != CELLS-1) begin
        ex_rd_addr = cell_addr + 1'b1;
    end
    if (row != CELLS-1) begin
        ey_adj_rd_addr  = cell_addr + CELLS;
        engine_ey_right = ey_adj_dout;
    end
    engine_ey_left = ey_rd_dout;
end
```

**After:**
```systemverilog
engine_ey_left    = prev_ey;
engine_ey_right   = ey_rd_dout;
engine_bz_left_ey = prev_bz;
engine_bz_left_ex = prev_bz;

if (counter < GRID_SIZE) begin
    ey_rd_addr = cell_addr;
    ex_rd_addr = cell_addr;
    bz_rd_addr = cell_addr;
    if (row != 0) begin
        bz_adj_rd_addr    = cell_addr - CELLS;
        engine_bz_left_ey = bz_adj_dout;
    end
end else begin
    bz_rd_addr = cell_addr;
    ey_rd_addr = cell_addr;
    if (column != CELLS-1) begin
        ex_rd_addr = cell_addr + 1'b1;
    end
    if (row != CELLS-1) begin
        ey_adj_rd_addr  = cell_addr + CELLS;
        engine_ey_right = ey_adj_dout;
    end
    engine_ey_left = ey_rd_dout;
end
```

Three things changed in the merged E-field phase (first branch):
1. `ex_rd_addr = cell_addr` is now set here (previously only in phase 2)
2. `engine_bz_left_ey` gets `bz_adj_dout` when `row != 0` — the row-above
   Bz value for the Ey curl
3. `engine_bz_left_ex` stays as `prev_bz` always — the column-left Bz value
   for the Ex curl, which is already in the register from the previous cycle

The old phase 2 (Ex-only reads) disappears entirely. The old phase 3 (Bz)
becomes the new phase 2 (else branch) unchanged.

---

### 2e. `always_ff` — write enables and termination

Both `ey_we` and `ex_we` must fire together during phase 1, and the counter
terminates at `TWO_GRID_SIZE` instead of `THREE_GRID_SIZE`.

**Before:**
```systemverilog
if (counter == THREE_GRID_SIZE - 1) solver_done <= 1'b1;

if (counter < GRID_SIZE) begin
    ey_we <= write_valid;
    if (wr_row == 0 || wr_row == CELLS-1) ey_wr_data <= '0;
    else if (source_valid && wr_cell == source_addr) ey_wr_data <= source_in;
    else ey_wr_data <= engine_ey_new;
end

else if (counter < TWO_GRID_SIZE) begin
    ex_we <= write_valid;
    if (wr_column == 0 || wr_column == CELLS-1) ex_wr_data <= '0;
    else ex_wr_data <= engine_ex_new;

end else if (counter < THREE_GRID_SIZE) begin
    bz_we      <= write_valid;
    bz_wr_data <= engine_bz_new;
end

if (counter < THREE_GRID_SIZE) counter <= counter + 1'b1;
```

**After:**
```systemverilog
if (counter == TWO_GRID_SIZE - 1) solver_done <= 1'b1;

if (counter < GRID_SIZE) begin
    ey_we <= write_valid;
    ex_we <= write_valid;
    if (wr_row == 0 || wr_row == CELLS-1) ey_wr_data <= '0;
    else if (source_valid && wr_cell == source_addr) ey_wr_data <= source_in;
    else ey_wr_data <= engine_ey_new;
    if (wr_column == 0 || wr_column == CELLS-1) ex_wr_data <= '0;
    else ex_wr_data <= engine_ex_new;
end else begin
    bz_we      <= write_valid;
    bz_wr_data <= engine_bz_new;
end

if (counter < TWO_GRID_SIZE) counter <= counter + 1'b1;
```

Note that the Ey and Ex boundary conditions use different signals: Ey uses
`wr_row` (top/bottom walls), Ex uses `wr_column` (left/right walls). They
operate on the same `wr_cell` address but check independent conditions, so
they do not interfere with each other.

---

## Change 3 — `tb_fdtd_solver.sv`

Test 1 checks the exact cycle count for `solver_done`. The expected value
changes from `3*GRID` to `2*GRID`:

**Before:**
```systemverilog
$display("  solver_done after %0d cycles (expected %0d)", cycles_taken, 3*GRID);
if (cycles_taken !== 3*GRID) begin
```

**After:**
```systemverilog
$display("  solver_done after %0d cycles (expected %0d)", cycles_taken, 2*GRID);
if (cycles_taken !== 2*GRID) begin
```

---

## What does not change

- `Ey.sv`, `Ex.sv`, `Bz.sv` — no changes, they only receive ca/cb/old/neighbours
- `pml.sv` — no changes
- All BRAM-facing ports on `fdtd_solver` — no changes
- `solver_enable`, `solver_done` ports — no changes
- `fdtd_solver_bd_adapter.v` — no changes, it only sees `solver_done`
- The `write_valid = (cell_addr >= 3)` guard — unchanged, the 3-cycle
  pipeline delay applies equally to both Ey and Ex in the merged pass

---

## Cycle count summary

```
                Before          After
Ey pass         36,864          36,864  (merged)
Ex pass         36,864               0  (eliminated as separate phase)
Bz pass         36,864          36,864
                ──────          ──────
Total          110,592          73,728

solver_done fires at counter = 110,591  →  73,727
```
