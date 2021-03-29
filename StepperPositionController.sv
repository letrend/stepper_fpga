module StepperPositionController (
    input clk,
    input reset,
    input write,
    input [4:0] address,
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
      pterm, iterm, term_sum, result, pos, pos_offset, ramp_up_limit, ramp_up_threshold;

  assign dir = result>=0;

  assign readdata = (
        (address==5'h0)?setpoint:
        (address==5'h1)?Kp:
        (address==5'h2)?Ki:
        (address==5'h3)?deadband:
        (address==5'h4)?integralMax:
        (address==5'h5)?outputMax:
        (address==5'h6)?error:
        (address==5'h7)?position:
        (address==5'h8)?pterm:
        (address==5'h9)?iterm:
        (address==5'hA)?term_sum:
        (address==5'hB)?result:
        (address==5'hC)?pos_offset:
        (address==5'hD)?endswitch: // endswitch is active-low
        (address==5'hE)?ticks_per_millisecond:
        (address==5'hF)?enable:
        (address==5'h10)?MS:
        (address==5'h11)?result_freq:
        (address==5'h12)?ramp_up_limit:
        (address==5'h13)?ramp_up_threshold:
        0
        );

  always @ ( posedge clk, posedge reset ) begin: AVALON_INTERFACE
  if(reset)begin
    outputMax<=12000;
    ramp_up_limit<=6000;
    ramp_up_threshold<=30;
  end else begin
    if(write)begin
      case (address)
        5'h0: setpoint <= writedata;
        5'h1: Kp <= writedata;
        5'h2: Ki <= writedata;
        5'h3: deadband <= writedata;
        5'h4: integralMax <= writedata;
        5'h5: outputMax <= writedata;
        5'hC: pos_offset <= pos;
        5'hF: enable <= !writedata[0]; // enable is active-low
        5'h10: MS <= writedata[1:0]; // mode select
        5'h12: ramp_up_limit <= writedata;
        5'h13: ramp_up_threshold <= writedata;
      endcase
      end
    end
  end

  always @ ( posedge clk ) begin: PI_CONTROLLER
      position <= pos-pos_offset;
      error = (setpoint-position);
      pterm = (Kp * error);
  		iterm = iterm + (Ki * error); //add to the integral
  		if (iterm > integralMax) begin
  			iterm = integralMax;
  		end else if (iterm < -integralMax) begin
  			iterm = -integralMax;
  		end
      term_sum = pterm + iterm;
      if(term_sum>deadband || term_sum<-deadband)begin
          result = term_sum;
      end else begin
        result = 0;
      end
  end

  integer result_freq;

  always @ ( posedge reset, posedge clk ) begin: RAMP_UP_CONTROL
  if( reset )begin
    step_freq_hz <= 0;
  end else begin
    if(result>ramp_up_limit && ticks_per_millisecond>ramp_up_threshold)begin
      result_freq <= outputMax;
    end else if(result<-ramp_up_limit && ticks_per_millisecond<-ramp_up_threshold)begin
      result_freq <= -outputMax;
    end else begin
      if(result>ramp_up_limit)begin
        result_freq <= ramp_up_limit;
      end else if(result<-ramp_up_limit)begin
        result_freq <= -ramp_up_limit;
      end else begin
        result_freq <= result;
      end
    end

    if(endswitch)begin // endswitch is active low
      step_freq_hz <= result_freq>=0?result_freq:-result_freq;
    end else begin
      step_freq_hz <= result_freq>=0?0:-result_freq; // dont allow going further than endswitch
    end
  end
  end

  integer pos_prev, clk_counter_millisecond, ticks_per_millisecond;

  always @ ( posedge clk ) begin: TICKS_PER_MILLISECOND
    clk_counter_millisecond <= clk_counter_millisecond+1;
    if(clk_counter_millisecond==(CLOCK_FREQ_HZ/1000-1))begin
      clk_counter_millisecond <= 0;
      ticks_per_millisecond <= (position-pos_prev);
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
