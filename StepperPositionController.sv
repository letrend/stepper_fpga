module StepperPositionController (
    input clk,
    input reset,
    input write,
    input [3:0] address,
    input signed [31:0] writedata,
    input read,
    output signed [31:0] readdata,
    output step,
    output dir,
    input A,
    input B,
    input I,
    output reg enable,
    output reg [1:0] MS,
    input endswitch
  );

  parameter CLOCK_FREQ_HZ = 50_000_000;

  reg slow_clk;
  integer clk_counter;
  integer step_freq_hz;

  assign step = slow_clk;

  always @ ( posedge reset, posedge clk ) begin
    if(reset)begin
      clk_counter <= 0;
      slow_clk <= 0;
    end else begin
      if(step_freq_hz>0)begin
        if(clk_counter==0)begin
          clk_counter <= CLOCK_FREQ_HZ/step_freq_hz/2-1;
          slow_clk <= !slow_clk;
        end else begin
          clk_counter <= clk_counter - 1;
        end
      end
    end
  end

  integer setpoint, Kp, Ki, deadband, integralMax, outputMax, error, position,
      pterm, iterm, result, pos, pos_offset;

  assign dir = result>=0;

  assign readdata = (
        (address==4'h0)?setpoint:
        (address==4'h1)?Kp:
        (address==4'h2)?Ki:
        (address==4'h3)?deadband:
        (address==4'h4)?integralMax:
        (address==4'h5)?outputMax:
        (address==4'h6)?error:
        (address==4'h7)?position:
        (address==4'h8)?pterm:
        (address==4'h9)?iterm:
        (address==4'hA)?result:
        (address==4'hB)?pos_offset:
        (address==4'hC)?endswitch:
        (address==4'hD)?ticks_per_millisecond:
        (address==4'hE)?enable:
        (address==4'hF)?MS:
        0
        );

  always @ ( posedge clk ) begin: AVALON_INTERFACE
    if(write)begin
      case (address)
        4'h0: setpoint <= writedata;
        4'h1: Kp <= writedata;
        4'h2: Ki <= writedata;
        4'h3: deadband <= writedata;
        4'h4: integralMax <= writedata;
        4'h5: outputMax <= writedata;
        4'hB: pos_offset <= pos;
        4'hE: enable <= !writedata[0]; // enable is active-low
        4'hF: MS <= writedata[1:0]; // mode select
      endcase
      end
  end

  always @ ( posedge reset, posedge clk ) begin: PI_CONTROLLER
    if( reset )begin
      step_freq_hz = 0;
    end else begin
      position <= pos-pos_offset;
      error = (setpoint-position);
      pterm = (Kp * error);
  		iterm = iterm + (Ki * error); //add to the integral
  		if (iterm > integralMax) begin
  			iterm = integralMax;
  		end else if (iterm < -integralMax) begin
  			iterm = -integralMax;
  		end
      result = pterm + iterm;
      if(result>deadband || result<-deadband)begin
        if(result>outputMax)begin
          result = outputMax;
        end else if(result<-outputMax)begin
          result = -outputMax;
        end
        if(endswitch)begin // endswitch is active low
          step_freq_hz = result>=0?result:-result;
        end else begin
          step_freq_hz = result>=0?0:-result; // dont allow going further than endswitch
        end
      end else begin
        step_freq_hz = 0;
      end
    end
  end

  integer pos_prev, clk_counter_millisecond, ticks_per_millisecond;

  always @ ( posedge clk ) begin: TICKS_PER_MILLISECOND
    clk_counter_millisecond <= clk_counter_millisecond+1;
    if(clk_counter_millisecond==(CLOCK_FREQ_HZ/1000-1))begin
      clk_counter_millisecond <= 0;
      ticks_per_millisecond <= (pos_prev-position);
      pos_prev <= position;
    end
  end

  reg encoder_movement_direction;

  quadrature_decoder #(0) quad_counter(
     .clk(clk),
     .a(A),
     .b(B),
     .direction(encoder_movement_direction),
     .position(pos)
   );


endmodule //Stepper
