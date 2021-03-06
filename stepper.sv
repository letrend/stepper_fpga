module Stepper (
    input clk,
    input reset,
    input write,
    input [1:0] address,
    input unsigned [31:0] writedata,
    input read,
    output unsigned [31:0] readdata,
    output step,
    output dir,
    output reg enable
  );

  parameter CLOCK_FREQ_HZ = 50_000_000;

  reg waitFlag;
  assign waitrequest = (waitFlag && (read||write));
  localparam  IDLE = 0, STEP = 1;
  reg [2:0] state;

  reg slow_clk;
  integer clk_counter;
  integer step_freq_hz;

  assign step = slow_clk;

  always @ ( posedge reset, posedge clk ) begin
    if(reset)begin
      clk_counter <= 0;
      slow_clk <= 0;
    end else begin
      if(state==STEP)begin
        if(clk_counter==0)begin
          clk_counter <= CLOCK_FREQ_HZ/step_freq_hz/2-1;
          slow_clk <= !slow_clk;
        end else begin
          clk_counter <= clk_counter - 1;
        end
      end
    end
  end

  reg reset_step_counter;
  integer step_counter, target_steps;
  wire direction;
  assign direction = target_steps>=0;
  assign dir = direction;
  always @ ( posedge reset_step_counter, negedge slow_clk ) begin
    if(reset_step_counter)begin
      step_counter <= 0;
    end else begin
      if(direction)begin
        step_counter <= step_counter+1;
      end else begin
        step_counter <= step_counter-1;
      end
    end
  end

  assign state = (step_counter!=target_steps?STEP:IDLE);
  assign readdata = step_counter;

  always @ ( posedge reset, posedge clk ) begin: AVALON_INTERFACE
    if(reset)begin
      step_freq_hz <= 16_000;
    end else begin
      if(step_counter==0)begin // wait for it to apply
        reset_step_counter <= 0;
      end
      if(write)begin
        if(address==0)begin
          reset_step_counter <= 1;
          target_steps <= writedata;
        end else if (address==1)begin
          step_freq_hz <= writedata;
        end else if (address==2)begin
          enable <= writedata;
        end
      end
    end
  end


endmodule //Stepper
