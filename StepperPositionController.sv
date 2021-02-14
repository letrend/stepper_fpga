module StepperPositionController (
    input clk,
    input reset,
    input write,
    input [3:0] address,
    input unsigned [31:0] writedata,
    input read,
    output unsigned [31:0] readdata,
    output step,
    output reg dir,
    input A,
    input B,
    input I,
    output reg enable
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

  integer setpoint, Kp, Ki, deadband, intergralMax, error, position,
      pterm, iterm, outputMax, result;

  // assign readdata = target_position;

  always @ ( posedge clk ) begin: AVALON_INTERFACE    
    if(write)begin
      case (address)
        4'h0: setpoint <= writedata;
        4'h1: Kp <= writedata;
        4'h2: Ki <= writedata;
        4'h3: deadband <= writedata;
        4'h4: intergralMax <= writedata;
      endcase
      end
  end

  always @ ( posedge reset, posedge clk ) begin: PI_CONTROLLER
    if( reset )begin
      step_freq_hz <= 0;
    end else begin
      error = (setpoint-position);
      pterm = (Kp * error);
  		iterm = iterm + (Ki * error); //add to the integral
  		if (iterm > intergralMax) begin
  			iterm = intergralMax;
  		end else if (iterm < -intergralMax) begin
  			iterm = -intergralMax;
  		end
      result = pterm + iterm;
      if(result>outputMax)begin
        result = outputMax;
      end else if(result<-outputMax)begin
        result = -outputMax;
      end else begin
        if(result>=0)begin
          dir <= 0;
          step_freq_hz <= result;
        end else begin
          dir <= 1;
          step_freq_hz <= -result;
        end
      end
    end
  end

  reg encoder_movement_direction;

  quadrature_decoder #(0) quad_counter(
     .clk(clk),
     .a(A),
     .b(B),
     .direction(encoder_movement_direction),
     .position(position)
   );


endmodule //Stepper
