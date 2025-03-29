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



module NRZI_decode (
  input logic stream,
  input logic ready,
  input logic clock, reset_n,
  output logic out
);
  logic prev;
  always_comb begin
    out = prev == stream;
  end
  always_ff @(posedge clock, negedge reset_n) begin
    if (reset_n == 0) begin
      prev <= 1'b1;
    end else begin
      if (ready) begin
        prev <= stream;
      end
    end
  end
endmodule : NRZI_decode

module BitStuffer 
#(WIDTH = 24)
(
  input logic [WIDTH-1 : 0] parallelIn,
  input logic ready,
  input logic clock, reset_n,
  output logic finished,
  output logic out
);

  logic [$clog2(WIDTH):0] index;
  logic [$clog2(WIDTH):0] onesCount;
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
          if (index == WIDTH & onesCount != 6) begin 
            finished <= 1;
          end
        end  
      end
    end
  end
endmodule : BitStuffer

module BitUnstuffer 
#(WIDTH = 24)
(
  input logic ready,
  input logic in,
  input logic clock, reset_n,
  output logic finished,
  output logic [WIDTH-1 : 0] parallelOut
);

  logic [$clog2(WIDTH):0] index;
  logic [$clog2(WIDTH):0] onesCount;
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
        end
        else begin 
          parallelOut[index] <= in;
          if (in) begin 
            onesCount <= onesCount + 1;
          end
          else begin 
            onesCount <= 0;
          end
          index <= index + 1;
          if (index == WIDTH & onesCount != 6) begin 
            finished <= 1;
          end
        end  
      end
    end
  end
endmodule : BitUnstuffer

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


module CRC16
#(WIDTH = 64)
(
  input logic [WIDTH-1:0] parallelIn,
  input logic ready,
  input logic clock,
  input logic reset_n,
  output logic done,
  output logic [15:0] out
);
  logic [$clog2(WIDTH):0] index;
  logic stillGoing;
  always_ff @(posedge clock, negedge reset_n) begin
    if (reset_n == 0) begin 
      out <= 16'hFFFF;
      index <= WIDTH-1;
      stillGoing <= 1;
    end
    else begin
      if (ready & stillGoing) begin 
        out[0] <= out[15] ^ parallelIn[index];
        out[1] <= out[0];
        out[2] <= out[15] ^ parallelIn[index] ^ out[1];
        out[3] <= out[2];
        out[4] <= out[3];
        out[5] <= out[4];
        out[6] <= out[5];
        out[7] <= out[6];
        out[8] <= out[7];
        out[9] <= out[8];
        out[10] <= out[9];
        out[11] <= out[10];
        out[12] <= out[11];
        out[13] <= out[12];
        out[14] <= out[13];
        out[15] <= out[15] ^ parallelIn[index] ^ out[14];
        index <= index-1;
      end
      if (index == 0) begin 
        stillGoing <= 0;
      end
    end
  end
endmodule : CRC16

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


module InOutPacket 
#(TYPE = 0) //0 for OUT, 1 for IN
(
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
  logic [15:0] parallelIn;
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
  logic [7:0] PID, PID_reverse;
  logic [15:0] Pattern;
  logic [6:0] address, reverse_address;
  logic [3:0] endpoint, reverse_endpoint;
  //////////////////////////

  BitStuffer #(16) BS (.parallelIn(parallelIn),
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
  assign PID = TYPE ? 8'b0110_1001 : 8'b1110_0001;
  assign PID_reverse = TYPE ? 8'b1001_0110 : 8'b1000_0111;

  assign Pattern[15:8] = SYNC_pattern;
  assign Pattern[7:0] = PID_reverse;
  

  assign address = `DEVICE_ADDR;
  assign endpoint = `ADDR_ENDP;  

  // assign parallelIn[7:0] = PID;

  assign parallelIn[6:0] = address;
  assign parallelIn[10:7] = endpoint;
  assign parallelIn[15:11] = ~CRC_reverse_out;
  
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
      SYNC_index <= 4'd15; //start out by sending the SYNC at MSB
      SYNC_done <= 1'b0;
      SE0_count <= 4'b0;
      enable <= 0;
      finishedOut <= 0;
      sendSE0 <= 0;
    end
    else if (startOut) begin 
      CRC_ready <= 1; 
      NRZI_ready <= 1;
      if (~SYNC_done) begin
        enable <= 1;
        NRZI_stream <= Pattern[SYNC_index];
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
    else begin 
      //restart the entire sequence so we can
      //send more packets
      enable <= 0;
      SYNC_index <= 4'd15; 
      SYNC_done <= 1'b0;
      SE0_count <= 4'b0;
      enable <= 0;
      finishedOut <= 0;
      sendSE0 <= 0;
    end
  end
endmodule : InOutPacket

module DataPacket 
(
  input logic startOut,
  input logic [63:0] data,
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
  logic [79:0] parallelIn;
  logic ready, out, finished;
  ///////////////////////////

  //NRZI logic
  logic NRZI_stream;
  logic NRZI_ready;
  logic NRZI_out;
  ///////////////////////////

  //CRC logic
  logic [63:0] CRC_in;
  logic CRC_ready;
  logic [15:0] CRC_out, CRC_reverse_out;
  ///////////////////////////

  //OTHER PACKET INFO
  logic [7:0] SYNC_pattern;
  logic [7:0] PID, PID_reverse;
  logic [15:0] Pattern;
  //////////////////////////

  BitStuffer #(80) BS (.parallelIn(parallelIn),
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

  CRC16 #(64) C (.parallelIn(CRC_in),
                .ready(CRC_ready),
                .clock(clock),
                .reset_n(reset_n),
                .out(CRC_out));

  //Reverse
  Reverse #(64) DATA_reverse (.in(data), .out(CRC_in));
  Reverse #(16) CRC_reverse (.in(CRC_out), .out(CRC_reverse_out));
  //////////////////////////

  //PID assign 
  assign SYNC_pattern = 8'b0000_0001;
  assign PID = 8'b1100_0011;
  assign PID_reverse = 8'b1100_0011;

  assign Pattern[15:8] = SYNC_pattern;
  assign Pattern[7:0] = PID_reverse;
  

  assign parallelIn[63:0] = data;
  assign parallelIn[79:64] = ~CRC_reverse_out;
  
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
      SYNC_index <= 4'd15; //start out by sending the SYNC at MSB
      SYNC_done <= 1'b0;
      SE0_count <= 4'b0;
      enable <= 0;
      finishedOut <= 0;
      sendSE0 <= 0;
    end
    else if (startOut) begin 
      CRC_ready <= 1; 
      NRZI_ready <= 1;
      if (~SYNC_done) begin
        enable <= 1;
        NRZI_stream <= Pattern[SYNC_index];
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
    else begin 
      //restart the entire sequence so we can
      //send more packets
      enable <= 0;
      SYNC_index <= 4'd15; 
      SYNC_done <= 1'b0;
      SE0_count <= 4'b0;
      enable <= 0;
      finishedOut <= 0;
      sendSE0 <= 0;
    end
  end
endmodule : DataPacket



module DataInPacket (
  USBWires wires,
  input logic start,
  input logic clock, reset_n,
  output logic incorrect,
  output logic finished
);
  //Sync logic
  logic [5:0] SYNC_count;
  logic readingSync;
  logic badStream;

  //PID logic 
  logic [5:0] PID_count;
  logic readingPID;
  logic [8:0] PID;
  
  //NRZI logic
  logic stream; 
  logic NRZI_ready;
  logic NRZI_in;
  logic NRZI_output;

  //Bit stuffer logic
  logic BU_ready;
  logic BU_finished;
  logic [79:0] BU_out;

  //EOP logic
  logic readingEOP;
  logic [5:0] SE0_count;

  //CRC check logic
  logic CRC_ready;
  logic [15:0] CRC_extract, CRC_negated_final,CRC_final;
  logic CRC_done;
  logic [15:0] CRC_out;
  logic [63:0] CRC_in;

  //Set sync logic
  always_comb begin 
    if (wires.DP == 0 && wires.DM == 1) begin 
      stream = 0; //restart count
      badStream = 0;
    end
    else if (wires.DP == 1 && wires.DM == 0) begin 
      stream = 1;
      badStream = 0;
    end
    else begin
      stream = 1;
      badStream = 1;
    end
  end


  //Set CRC logic
  assign CRC_extract = BU_out[79:64];
  Reverse #(16) CRC_reverse (.in(CRC_extract), .out(CRC_negated_final));
  assign CRC_final = ~CRC_negated_final;
  Reverse #(64) data_reverse(.in(BU_out[63:0]), .out(CRC_in));

  NRZI_decode #(80) ND (
  .stream(stream),
  .ready(NRZI_ready),
  .clock(clock), 
  .reset_n(reset_n),
  .out(NRZI_output));

  BitUnstuffer #(80) BU (
  .ready(BU_ready),
  .in(NRZI_output),
  .clock(clock), 
  .reset_n(reset_n),
  .finished(BU_finished),
  .parallelOut(BU_out));

  CRC16 #(64) (
  .parallelIn(CRC_in),
  .ready(CRC_ready),
  .clock(clock),
  .reset_n(reset_n),
  .done(CRC_done),
  .out(CRC_out));

  always_ff @(posedge clock, negedge reset_n) begin 
    if (reset_n) begin 
      SYNC_count <= 0;
      readingSync <= 1;
      NRZI_ready <= 0;

      PID_count <= 0;
      readingPID <= 0;

      BU_ready <= 0;

      readingEOP <= 1;
      SE0_count <= 1;

      incorrect <= 0;
      finished <= 0;
    end
    if (!finished && start) begin 
      NRZI_ready <= 1;
      if (readingSync) begin 
        if (SYNC_count < 7) begin 
          if (NRZI_output == 0 & ~badStream) begin 
            SYNC_count <= SYNC_count+1; 
          end
          else begin 
            SYNC_count <= 0; //restart count
          end
        end
        else if (SYNC_count == 7) begin 
          if (NRZI_output == 1 & ~badStream) begin 
            readingSync <= 0; //no longer reading SYNC
            readingPID <= 1;
          end
          else begin 
            SYNC_count <= 0;
          end
        end
      end
      else if (readingPID) begin 
        if (PID_count < 8) begin 
          if (~badStream) begin 
            PID[PID_count] <= NRZI_output;
            PID_count <= PID_count + 1;
          end
          else begin 
            incorrect <= 1;
            finished <= 1;
          end
        end
        else begin 
          readingPID <= 0;
        end
      end
      else if (!BU_finished) begin 
        if ((PID[0] == ~PID[4]) && (PID[1] == ~PID[5]) && (PID[2] == ~PID[7]) && (PID[3] == ~PID[7])) begin 
          incorrect <= 1;
          finished <= 1;
        end
        else begin 
          incorrect <= 1;
          finished <= 1;
        end
      end
      else if (BU_finished && readingEOP) begin 
        if (SE0_count < 2) begin 
          if (wires.DP == 0 && wires.DM == 0) begin 
            SE0_count <= SE0_count + 1;
          end
          else begin 
            incorrect <= 1;
            finished <= 1;
          end
        end
        else begin 
          if (wires.DP != 1'bz || wires.DM != 1'bz) begin
            incorrect <= 1;
            finished <= 1;
          end
          else begin 
            readingEOP <= 0;
          end
        end
      end
      else if (!CRC_done) begin
        //keep computing
      end
      else begin 
        if (CRC_out != CRC_final) begin 
          incorrect <= 1;
          finished <= 1;
        end
      end
    end
  end
endmodule : DataInPacket





module USBHost (
  USBWires wires,
  input logic clock, reset_n
);

logic startOut, startIn, startDataIn;
logic incorrect;
logic finishedOut, finishedIn, finishedDataIn;

logic startData, finishedData;
logic [63:0] data;

InOutPacket #(0) OUT (.startOut(startOut), .clock(clock), .reset_n(reset_n), .finishedOut(finishedOut), .wires(wires));
InOutPacket #(1) IN  (.startOut(startIn), .clock(clock), .reset_n(reset_n), .finishedOut(finishedIn), .wires(wires));
DataPacket Test (.startOut(startData), .data(data), .clock(clock), .reset_n(reset_n), .finishedOut(finishedData), .wires(wires));
DataInPacket Test2 (.wires(wires), .start(startDataIn), .clock(clock), .reset_n(reset_n), .incorrect(incorrect), .finished(finishedDataIn));
//PRELAB task sends an Out packet
task prelabRequest();  
  // startOut = 1;
  // wait(finishedOut);
  // startOut = 0;
  // @(posedge clock);
  // $display("%b", IN.PID);
  data = 64'h0000FFFF0000FFFF;
  startData = 1;
  startDataIn = 1;
  // wait(finishedData);
  $display("parallel in %b", Test.parallelIn);
  while (!finishedData) begin 
    @(posedge clock);
    // $display("%b, %b", Test.NRZI_stream, Test.CRC_out);
    $display("NRZI out: %b, REF: %b", Test2.NRZI_output,Test.NRZI_stream);

  end
  startData = 0;
  startDataIn = 1;
  $display("FINISHED SENDING PACKET");
  $display("%h", Test2.BU_out);
  $display("%b", finishedDataIn);
  $display("%d %b", Test2.SYNC_count, Test2.readingSync);
  for (int i = 0; i < 10; i++) begin
    @(posedge clock);
    $display("POOOOOOO");
    $display("%h", Test2.BU_out);
  end
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

  startOut = 1;
  wait(finishedOut);
  startOut = 0;
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







