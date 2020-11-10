`timescale 10ns/10ns
module testbench_stepper;
  reg clk, reset, write, read;
  reg [31:0] writedata;
  wire [31:0] readdata;
  wire step, dir;

  Stepper DUT(clk,reset,
    write,writedata,read,readdata,
    step, dir);

  initial
  begin
    clk = 0;
    reset = 0;
    write = 0;
    read = 0;
    #2
    reset = 1;
    #2
    reset = 0;
    #2
    // step test positive direction
    read = 0;

    write = 1;
    writedata = 10;
    #2
    write = 0;
    #200
    write = 1;
    writedata = -10;
    #2
    write = 0;
  end

  always
    #1 clk = !clk;

endmodule // testbench_TCM4671
