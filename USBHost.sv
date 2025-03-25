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
    end
    else begin 
      if (ready) begin 
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

module Reverse_7
#(WIDTH = 7)
(input logic [WIDTH-1:0] in,
 output logic [WIDTH-1:0] out);
  assign out[0] = in[6];
  assign out[1] = in[5];
  assign out[2] = in[4];
  assign out[3] = in[3];
  assign out[4] = in[2];
  assign out[5] = in[1];
  assign out[6] = in[0];
endmodule : Reverse_7

module Reverse_4
#(WIDTH = 4)
(input logic [WIDTH-1:0] in,
 output logic [WIDTH-1:0] out);
  assign out[0] = in[3];
  assign out[1] = in[2];
  assign out[2] = in[1];
  assign out[3] = in[0];
endmodule : Reverse_4



module USBHost (
  USBWires wires,
  input logic clock, reset_n
);

logic sendingEOP, sendingSE0, sendingIdle;



logic [23:0] parallelIn;
logic ready, out, finished;

logic NRZI_stream;
logic NRZI_ready;
logic NRZI_out;

logic [10:0] CRC_in;
logic CRC_ready;
logic [4:0] CRC_out;


logic [7:0] SYNC_pattern;
logic [7:0] PID;
logic [6:0] address, reverse_address;
logic [3:0] endpoint, reverse_endpoint;


assign address = `DEVICE_ADDR;
assign endpoint = `DATA_ENDP;

typedef enum logic [1:0]
  {BS_J = 2'b10, BS_K = 2'b01, BS_SE0 = 2'b00, BS_SE1 = 2'b11, BS_NC = 2'bzz}
  bus_state_t;


assign {wires.DP, wires.DM} = (~sendingIdle & NRZI_out) ? BS_J : BS_NC;
// assign {wires.DP, wires.DM} = (~sendingIdle & ~NRZI_out) ? BS_K : BS_NC;

// assign {wires.DP, wires.DM} = (sendingSE0) ? BS_SE0 : BS_NC;
// assign {wires.DP, wires.DM} = (sendingIdle) ? BS_NC : BS_NC;


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



Reverse_7 address_reverse (.in(address), .out(reverse_address));
Reverse_4 endpoint_reverse (.in(endpoint), .out(reverse_endpoint));


assign parallelIn[7:0] = PID;
assign parallelIn[14:8] = address;
assign parallelIn[18:15] = endpoint;
assign parallelIn[23:19] = ~CRC_out;

task prelabRequest();
  // device_address = DEVICE_ADDR;
  // data_endpoint = DATA_ENDP;  
  SYNC_pattern = 8'b0000_0001;

  PID = 8'b1110_0001;
  
  // address = 7'b0111010;
  // endpoint = 4'b1010;
  //SEND SYNC
  NRZI_ready = 0;
  CRC_ready = 1;
  CRC_in[10:4] = reverse_address;//7'b0101_110;//reverse_address;
  CRC_in[3:0] = reverse_endpoint;//4'b0101;//reverse_endpoint;

  sendingIdle = 1;
  for (int i = 7; i >= 0; i--) begin 
    sendingIdle <= 0;

    NRZI_ready <= 1;
    NRZI_stream <= SYNC_pattern[i];
    if (i == 0) begin
      ready = 1;
    end
    $display("%b, %b", wires.DP, wires.DM);
    @(posedge clock);
  end

  //SEND PACKET
  for (int i = 24; i >= 0; i--) begin 
    NRZI_stream <= out;
    @(posedge clock);
  end
  sendingEOP <= 1;
  sendingSE0 <= 1;
  @(posedge clock);
  sendingSE0 <= 1;
  @(posedge clock);
  sendingSE0 <= 0;
  sendingIdle <= 1;
  @(posedge clock);
  




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







