`include "USBPkg.pkg"

// Wrapper for USB bus states. Notice that enum Z can only be driven, not read
typedef enum logic [1:0]
  {BS_J = 2'b10, BS_K = 2'b01, BS_SE0 = 2'b00, BS_SE1 = 2'b11, BS_NC = 2'bzz}
  bus_state_t;


module NRZI (
  input logic stream,
  input logic ready,
  input logic clock, reset_n,
  output logic out
);
  logic prev;
  always_comb begin
    if (~stream) begin
      out = ~prev;
    end else begin
      out = prev;
    end
  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (reset_n == 0) begin
      prev <= 1'b1;
    end else begin
      if (ready) begin
        prev <= out;
      end
    end
  end
endmodule : NRZI

module BitStuffer 
#(WIDTH = 24)
(
  input logic [WIDTH-1 : 0] parallelIn,
  input logic ready,
  input logic clock, reset_n,
  output logic finished,
  output logic out
);

  logic [$clog2(WIDTH)-1:0] index;
  logic [$clog2(WIDTH)-1:0] onesCount;
  always_ff @(posedge clock, negedge reset_n) begin
    if (reset_n == 0) begin 
      onesCount <= 0;
      index <= 0;
      finished <= 0;
    end
    else begin 
      if (ready & ~finished) begin 
        if (onesCount == 6) begin
          onesCount <= 0;
          out <= 0;
        end
        else begin 
          out <= parallelIn[index];
          if (parallelIn[index] == 1) begin 
            onesCount <= onesCount + 1;
          end
          else begin 
            onesCount <= 0;
          end
          index <= index + 1;
          if (index == WIDTH) begin 
            finished <= 1;
          end
        end  
      end
    end
  end
endmodule : BitStuffer



module CRC5
#(WIDTH = 11)
(
  input logic [WIDTH-1:0] parallelIn,
  input logic ready,
  input logic clock,
  input logic reset_n,
  output logic done,
  output logic [4:0] out
);
  logic [$clog2(WIDTH)-1:0] index;
  logic stillGoing;
  always_ff @(posedge clock, negedge reset_n) begin
    if (reset_n == 0) begin 
      out <= 5'b1_1111;
      index <= WIDTH-1;
      stillGoing <= 1;
    end
    else begin
      if (ready & stillGoing) begin 
        out[0] <= out[4] ^ parallelIn[index];
        out[1] <= out[0];
        out[2] <= out[4] ^ parallelIn[index] ^ out[1];
        out[3] <= out[2];
        out[4] <= out[3];
        index <= index-1;
      end
      if (index == 0) begin 
        stillGoing <= 0;
      end
    end
  end
endmodule : CRC5

module Reverse
#(WIDTH = 7)
(input logic [WIDTH-1:0] in,
 output logic [WIDTH-1:0] out);
  genvar i;
  generate 
  for (i = 0; i < WIDTH; i++) begin : ReverseBits
    assign out[i] = in[WIDTH-i-1];
  end
  endgenerate
endmodule : Reverse


module OutPacket (
  input logic startOut,
  input logic clock, reset_n,
  output logic finishedOut,
  USBWires wires
);
  //tri state wire driving logic
  logic enable;
  logic [1:0] bus;
  logic sendSE0, idle;
  ///////////////////////////

  //bit stuffer logic
  logic [23:0] parallelIn;
  logic ready, out, finished;
  ///////////////////////////

  //NRZI logic
  logic NRZI_stream;
  logic NRZI_ready;
  logic NRZI_out;
  ///////////////////////////

  //CRC logic
  logic [10:0] CRC_in;
  logic CRC_ready;
  logic [4:0] CRC_out, CRC_reverse_out;
  ///////////////////////////

  //OTHER PACKET INFO
  logic [7:0] SYNC_pattern;
  logic [7:0] PID;
  logic [6:0] address, reverse_address;
  logic [3:0] endpoint, reverse_endpoint;
  //////////////////////////

  BitStuffer #(24) BS (.parallelIn(parallelIn),
                .ready(ready),
                .clock(clock), 
                .reset_n(reset_n), 
                .out(out),
                .finished(finished));

  NRZI N (.stream(NRZI_stream),
          .ready(NRZI_ready),
          .clock(clock), 
          .reset_n(reset_n),
          .out(NRZI_out));

  CRC5 #(11) C (.parallelIn(CRC_in),
                .ready(CRC_ready),
                .clock(clock),
                .reset_n(reset_n),
                .out(CRC_out));


  //Reverse
  Reverse #(7) address_reverse (.in(address), .out(reverse_address));
  Reverse #(4) endpoint_reverse (.in(endpoint), .out(reverse_endpoint));
  Reverse #(5) CRC_reverse (.in(CRC_out), .out(CRC_reverse_out));
  //////////////////////////

  assign CRC_in[10:4] = reverse_address; //Assign first part of CRC_in;
  assign CRC_in[3:0] = reverse_endpoint; //Assign second part of CRC_in;

  //PID assign 
  assign SYNC_pattern = 8'b0000_0001;
  assign PID = 8'b1110_0001;
  assign address = 7'd63;//`DEVICE_ADDR;
  assign endpoint = `DATA_ENDP;  

  assign parallelIn[7:0] = PID;
  assign parallelIn[14:8] = address;
  assign parallelIn[18:15] = endpoint;
  assign parallelIn[23:19] = ~CRC_reverse_out;
  
  //control variables
  logic [3:0] SYNC_index;  
  logic SYNC_done;

  logic [3:0] SE0_count;
  /////////////////////////////


  assign {wires.DP, wires.DM} = enable ? bus : BS_NC;

  always_comb begin 
    if (sendSE0) begin 
      bus = BS_SE0;
    end
    else begin 
      if (NRZI_out) begin 
        bus = BS_J;
      end
      else begin 
        bus = BS_K;
      end
    end
  end

  
  always_ff @(negedge reset_n, posedge clock) begin 
    if (~reset_n) begin 
      enable <= 0;
      SYNC_index <= 4'd7; //start out by sending the SYNC at MSB
      SYNC_done <= 1'b0;
      SE0_count <= 4'b0;
      enable <= 0;
      finishedOut <= 0;
    end
    else begin 
      if (startOut) begin 
        CRC_ready <= 1; 
        NRZI_ready <= 1;
        if (~SYNC_done) begin
          enable <= 1;
          NRZI_stream <= SYNC_pattern[SYNC_index];
          SYNC_index <= SYNC_index - 1;
          if (SYNC_index == 1) begin
            ready <= 1; //start bit stuffer
          end
          if (SYNC_index == 0) begin
            SYNC_done <= 1; 
          end
        end
        else if (~finished) begin 
          NRZI_stream <= out;
        end
        else if (SE0_count < 2) begin 
          SE0_count <= SE0_count + 1;
          sendSE0 <= 1;
        end
        else begin 
          idle <= 1;
          finishedOut <= 1;
          enable <= 0;
        end
      end
    end
  end
endmodule : OutPacket


module USBHost (
  USBWires wires,
  input logic clock, reset_n
);
logic start;
logic finished;
OutPacket DUT (.startOut(start), .clock(clock), .reset_n(reset_n), .finishedOut(finished), .wires(wires));

task prelabRequest();  
  start = 1;
  $display("%b", DUT.parallelIn);
  while (!finished) begin 
    @(posedge clock);
    $display("%b, %b, %b", wires.DP, wires.DM, DUT.NRZI_stream);
  end
endtask : prelabRequest

task readData
// Host sends mempage to thumb drive using a READ (OUT->DATA0->IN->DATA0)
// transaction, and then receives data from it. This task should return both the
// data and the transaction status, successful or unsuccessful, to the caller.
( input logic [15:0] mempage, // Page to write
  output logic [63:0] data, // Vector of bytes to write
  output logic success);

  data = 64'h0;
  success = 1'b0;

endtask : readData

task writeData
// Host sends mempage to thumb drive using a WRITE (OUT->DATA0->OUT->DATA0)
// transaction, and then sends data to it. This task should return the
// transaction status, successful or unsuccessful, to the caller.
( input logic [15:0] mempage, // Page to write
  input logic [63:0] data, // Vector of bytes to write
  output logic success);

  success = 1'b0;

endtask : writeData


endmodule : USBHost







