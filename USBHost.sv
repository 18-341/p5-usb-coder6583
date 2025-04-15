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
            end else begin
                prev <= 1'b1;
            end
        end
    end
endmodule : NRZI

module NRZIDecoder (
    input logic stream,
    input logic ready,
    input logic clock, reset_n,
    output logic out
);

    logic prev;

    always_comb begin
        if (prev != stream) begin
            out = 1'b0;
        end else begin
            out = 1'b1;
        end
    end

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            prev <= 1'b1;
        end else begin
            if (ready) begin
                if (prev != stream) begin
                    prev <= ~prev;
                end
            end else begin
                prev <= 1'b1;
            end
        end
    end
endmodule : NRZIDecoder

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
          if (index == WIDTH) begin
            finished <= 1;
          end
        end
      end else if (~ready) begin
        onesCount <= 0;
        index <= 0;
        finished <= 0;
      end
    end
  end
endmodule : BitStuffer

module BitUnstuffer (
    input logic clock, reset_n,
    input logic stream,
    input logic ready,
    output logic out, hold
);

    logic [2:0] count;
    logic counting, sixones;

    assign sixones = count == 3'd6;
    assign hold = sixones;
    assign out = stream;

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            count <= 3'b0;
            counting <= 1'b1;
        end else begin
            if (ready) begin
                if (counting) begin
                    if (stream) begin
                        count <= count + 3'd1;
                        if (sixones) begin
                            counting <= 1'b0;
                            count <= 3'b0;
                        end
                    end else begin
                        count <= 3'd0;
                    end
                end else begin
                    counting <= 1'b1;
                end
            end else begin
                count <= 3'd0;
                counting <= 1'b1;
            end
        end
    end
endmodule : BitUnstuffer

module BitStreamDecoder #(WIDTH = 32)
(
    input logic clock, reset_n,
    input logic stream,
    input logic ready,
    input logic hold,
    output logic [WIDTH-1:0] out,
    output logic finished
);

    logic [$clog2(WIDTH):0] idx;
    logic collecting, idxEnd;

    assign idxEnd = idx == WIDTH - 1;
    assign finished = idxEnd;

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            collecting <= 1'b1;
            out <= '0;
            idx <= '0;
        end else begin
            if (ready) begin
                if (~hold) begin
                    out[idx] <= stream;
                end
                if (collecting) begin
                    if (hold) begin
                        collecting <= 1'b0;
                    end else begin
                        if (idxEnd) begin
                            idx <= '0;
                        end else begin
                            idx <= idx + 1;
                        end
                    end
                end else begin
                    collecting <= 1'b1;
                    idx <= idx + 1;
                end
            end else begin
                idx <= '0;
                out <= '0;
                collecting <= 1'b1;
            end
        end
    end
endmodule : BitStreamDecoder

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
      done <= 1'b0;
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
        done <= 1'b1;
      end
    end
  end
endmodule : CRC5

module CRC5Checker #(WIDTH = 11) (
    input logic clock, reset_n,
    input logic stream,
    input logic ready,
    output logic done, correct
);
    logic [4:0] ffs;
    logic [$clog2(WIDTH):0] cnt;
    logic calc, is_done;

    logic cycleDone;
    assign cycleDone = cnt == WIDTH;
    assign done = is_done;
    assign correct = done & (ffs == 5'h0c);

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            cnt <= '0;
            ffs <= 5'b1_1111;
            calc <= 1'b0;
            is_done <= 1'b0;
        end else begin
            if (calc & ~cycleDone) begin
                ffs[0] <= ffs[4] ^ stream;
                ffs[1] <= ffs[0];
                ffs[2] <= ffs[4] ^ stream ^ ffs[1];
                ffs[3] <= ffs[2];
                ffs[4] <= ffs[3];
            end
            if (calc) begin
                if (~cycleDone) begin
                    cnt <= cnt + 1;
                end else begin
                    cnt <= '0;
                    calc <= 1'b0;
                    is_done <= 1'b1;
                end
            end else if (is_done) begin
                is_done <= 1'b0;
                if (ready) begin
                    calc <= 1'b1;
                end
            end else begin
                if (ready) begin
                    calc <= 1'b1;
                end
            end
        end
    end
endmodule : CRC5Checker

module CRC16 #(WIDTH = 64) (
    input logic clock, reset_n,
    input logic [WIDTH-1:0] parallelIn,
    input logic ready,
    output logic done,
    output logic [15:0] out
);
    logic [$clog2(WIDTH)-1:0] idx;
    logic calcing, is_done;
    logic idxEnd;

    assign idxEnd = idx == 0;
    assign done = is_done;

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n | ~ready) begin
            out <= 16'hffff;
            idx <= WIDTH - 1;
            calcing <= 1'b1;
            is_done <= 1'b0;
        end else begin
            if (ready & calcing) begin
                out[0] <= out[15] ^ parallelIn[idx];
                out[1] <= out[0];
                out[2] <= (out[15] ^ parallelIn[idx]) ^ out[1];
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
                out[15] <= (out[15] ^ parallelIn[idx]) ^ out[14];
                idx <= idx - 1;
            end
            if (idx == 0) begin
                calcing <= 1'b0;
                is_done <= 1'b1;
            end
        end
    end
endmodule : CRC16

module CRC16Checker #(WIDTH = 64) (
    input logic clock, reset_n,
    input logic stream,
    input logic ready,
    input logic hold,
    output logic done, correct
);
    logic [15:0] ffs;
    logic [$clog2(WIDTH):0] cnt;
    logic calc, is_done;

    logic cycleDone;
    assign cycleDone = cnt == WIDTH;
    assign done = is_done;
    assign correct = done & (ffs == 16'h800D);

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            cnt <= '0;
            ffs <= 16'hFFFF;
            calc <= 1'b0;
            is_done <= 1'b0;
        end else begin
            if (calc & ~cycleDone & ~hold) begin
                ffs[0] <= ffs[15] ^ stream;
                ffs[1] <= ffs[0];
                ffs[2] <= (ffs[15] ^ stream) ^ ffs[1];
                ffs[3] <= ffs[2];
                ffs[4] <= ffs[3];
                ffs[5] <= ffs[4];
                ffs[6] <= ffs[5];
                ffs[7] <= ffs[6];
                ffs[8] <= ffs[7];
                ffs[9] <= ffs[8];
                ffs[10] <= ffs[9];
                ffs[11] <= ffs[10];
                ffs[12] <= ffs[11];
                ffs[13] <= ffs[12];
                ffs[14] <= ffs[13];
                ffs[15] <= (ffs[15] ^ stream) ^ ffs[14];
            end
            if (calc) begin
                if (~cycleDone & ~hold) begin
                    cnt <= cnt + 1;
                end else if (cycleDone) begin
                    cnt <= '0;
                    calc <= 1'b0;
                    is_done <= 1'b1;
                end
            end else if (is_done) begin
                is_done <= 1'b0;
                if (ready) begin
                    calc <= 1'b1;
                end
            end else begin
                if (ready) begin
                    calc <= 1'b1;
                end
            end
        end
    end
endmodule : CRC16Checker

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

// TYPE: 0 is IN, 1 is OUT
module InOutPacket #(TYPE = 0) (
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
  assign PID = TYPE ? 8'b1110_0001 : 8'b0110_1001;
  assign PID_reverse = TYPE ? 8'b1000_0111 : 8'b1001_0110;

  assign Pattern[15:8] = SYNC_pattern;
  assign Pattern[7:0] = PID_reverse;


  assign address = `DEVICE_ADDR;
  assign endpoint = 4'd4;

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
    else begin
      if (startOut) begin
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
    end
  end
endmodule : InOutPacket

module DataPacket (
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
  Reverse #(64) data_reverse(.in(data), .out(CRC_in));
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
    else begin
      if (startOut) begin
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
    end
  end
endmodule : DataPacket

module DataInPacket (
  input logic startOut,
  input logic clock, reset_n,
  output logic finishedOut,
  output logic correct,
  output logic timedout,
  output logic [63:0] dataOut,
  USBWires wires
);
    logic prevStream, inStream, isSE0, valid;
    logic [1:0] bus;

    assign bus = {wires.DP, wires.DM};

    always_comb begin
        inStream = 1'b0;
        isSE0 = 1'b0;
        valid = 1'b0;
        if (~startOut) begin
            inStream = 1'b0;
            isSE0 = 1'b0;
            valid = 1'b0;
        end else if (bus === BS_SE0) begin
            isSE0 = 1'b1;
        end else if (bus === BS_J) begin
            inStream = 1'b1;
            valid = 1'b1;
        end else if (bus === BS_K) begin
            inStream =1'b0;
            valid = 1'b1;
        end
    end

    always_ff @(posedge clock, negedge reset_n) begin
        prevStream <= inStream;
    end

    // NRZI Decoder logic
    logic NRZI_stream, NRZI_ready, NRZI_out;
    NRZIDecoder nrzid(.stream(NRZI_stream), .ready(NRZI_ready),
                      .out(NRZI_out), .clock, .reset_n);

    // Bit Unstuffer logic
    logic BU_stream, BU_ready, BU_out, BU_hold;
    BitUnstuffer bu(.stream(BU_stream), .ready(BU_ready), .out(BU_out),
                    .hold(BU_hold), .clock, .reset_n);

    // BitStreamDecoder logic
    logic BSD_ready, BSD_done;
    logic [79:0] BSD_out;
    BitStreamDecoder #80 bsd(.stream(BU_out), .hold(BU_hold), .ready(BSD_ready),
                             .finished(BSD_done), .out(BSD_out),
                             .clock, .reset_n);

    // CRC16Checker logic
    logic CRCcheck_ready, CRCcheck_done, CRCcheck_correct;
    CRC16Checker #80 crc16c(.stream(BU_out), .hold(BU_hold),
                            .ready(CRCcheck_ready), .done(CRCcheck_done),
                            .correct(CRCcheck_correct), .clock, .reset_n);

    // State control logic
    logic waiting, readSYNC, readPID, readDATA, readEOP, isDone, isError;
    logic firstSYNC;

    assign NRZI_ready = firstSYNC | readSYNC | readPID | readDATA;
    assign BU_ready = readDATA;
    assign BSD_ready = readDATA;
    assign CRCcheck_ready = readDATA;

    logic [3:0] SYNCidx, PIDidx, EOPidx;
    logic [7:0] PIDreceived;
    assign firstSYNC = waiting && (prevStream == 1'b1 && inStream == 1'b0);

    assign finishedOut = isDone;
    assign correct = ~isError;

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            waiting <= 1'b1;
            readSYNC <= 1'b0;
            readPID <= 1'b0;
            readDATA <= 1'b0;
            readEOP <= 1'b0;
            SYNCidx <= 4'b0;
            PIDidx <= 4'b0;
            isDone <= 1'b0;
            isError <= 1'b0;
            PIDreceived <= 8'b0;
        end else begin
            if (waiting) begin
                if (startOut & prevStream == 1'b1 && inStream == 1'b0) begin
                    waiting <= 1'b0;
                    readSYNC <= 1'b1;

                    SYNCidx <= 4'd1;
                    NRZI_stream <= inStream;
                end
            end else if (readSYNC) begin
                NRZI_stream <= inStream;
                SYNCidx <= SYNCidx + 1;
                if (SYNCidx < 4'd7) begin
                    if (NRZI_out != 1'b0) begin
                        isError <= 1'b1;
                    end
                end
                if (SYNCidx == 4'd7) begin
                    if (NRZI_out != 1'b1) begin
                        isError <= 1'b1;
                    end
                    readSYNC <= 1'b0;
                    readPID <= 1'b1;
                    PIDidx <= 4'b0;
                end
            end else if (readPID) begin
                NRZI_stream <= inStream;
                PIDidx <= PIDidx + 1;
                PIDreceived[4'd7 - PIDidx] <= NRZI_out;
                if (PIDidx == 4'd7) begin
                    if (PIDreceived != 8'b1100_001x || NRZI_out != 1'b1) begin
                        isError <= 1'b1;
                    end
                    readPID <= 1'b0;
                    readDATA <= 1'b1;
                end
            end else if (readDATA) begin
                NRZI_stream <= inStream;
                BU_stream <= NRZI_out;
                if (BSD_done & CRCcheck_done) begin
                    readDATA <= 1'b0;
                    readEOP <= 1'b1;
                    EOPidx <= 4'b0;
                    if (~CRCcheck_correct) begin
                        isError <= 1'b1;
                    end
                end
            end else if (readEOP) begin
                EOPidx <= EOPidx + 4'b1;
                if (EOPidx <= 4'b1) begin
                    if (inStream != 1'b0) begin
                        isError <= 1'b1;
                    end
                end else if (EOPidx == 4'd2) begin
                    if (~isSE0) begin
                        isError <= 1'b1;
                    end
                    readEOP <= 1'b0;
                    isDone <= 1'b1;
                end
            end else if (isDone) begin
                waiting <= 1'b1;
            end
        end
    end

    // Timeout logic
    logic [8:0] timeoutCnt;
    logic is_TO;

    assign is_TO = timeoutCnt == 9'd255;

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            timeoutCnt <= '0;
        end else begin
            if (waiting) begin
                timeoutCnt <= timeoutCnt + 9'b1;
                if (is_TO) begin
                    timeoutCnt <= '0;
                end
            end else begin
                timeoutCnt <= '0;
            end
        end
    end
endmodule : DataInPacket

// TYPE
// 0: NAK, 1: ACK
module AckNakPacket #(TYPE = 0) (
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

  //NRZI logic
  logic NRZI_stream;
  logic NRZI_ready;
  logic NRZI_out;
  ///////////////////////////

  //OTHER PACKET INFO
  logic [7:0] SYNC_pattern;
  logic [7:0] PID, PID_reverse;
  logic [15:0] Pattern;
  //////////////////////////

  NRZI N (.stream(NRZI_stream),
          .ready(NRZI_ready),
          .clock(clock),
          .reset_n(reset_n),
          .out(NRZI_out));

  //PID assign
  assign SYNC_pattern = 8'b0000_0001;
  assign PID = TYPE ? 8'b1101_0010 : 8'b0101_1010;
  assign PID_reverse = TYPE ? 8'b0100_1011 : 8'b0101_1010;

  assign Pattern[15:8] = SYNC_pattern;
  assign Pattern[7:0] = PID_reverse;

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
    else begin
      if (startOut) begin
        NRZI_ready <= 1;
        if (~SYNC_done) begin
          enable <= 1;
          NRZI_stream <= Pattern[SYNC_index];
          SYNC_index <= SYNC_index - 1;
          if (SYNC_index == 0) begin
            SYNC_done <= 1;
          end
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
endmodule : AckNakPacket

module AckNakInPacket (
  input logic startOut,
  input logic clock, reset_n,
  output logic finishedOut,
  output logic isAck,
  output logic isNak,
  output logic timedout,
  USBWires wires
);
    logic prevStream, inStream, isSE0, valid;
    logic [1:0] bus;

    assign bus = {wires.DP, wires.DM};

    always_comb begin
        inStream = 1'b0;
        isSE0 = 1'b0;
        valid = 1'b0;
        if (~startOut) begin
            inStream = 1'b0;
            isSE0 = 1'b0;
            valid = 1'b0;
        end else if (bus === BS_SE0) begin
            isSE0 = 1'b1;
        end else if (bus === BS_J) begin
            inStream = 1'b1;
            valid = 1'b1;
        end else if (bus === BS_K) begin
            inStream =1'b0;
            valid = 1'b1;
        end
    end

    always_ff @(posedge clock, negedge reset_n) begin
        prevStream <= inStream;
    end

    // NRZI Decoder logic
    logic NRZI_stream, NRZI_ready, NRZI_out;
    NRZIDecoder nrzid(.stream(NRZI_stream), .ready(NRZI_ready),
                      .out(NRZI_out), .clock, .reset_n);

    // State control logic
    logic waiting, readSYNC, readPID, readEOP, isDone, isError;
    logic firstSYNC;

    assign NRZI_ready = firstSYNC | readSYNC | readPID;

    logic [3:0] SYNCidx, PIDidx, EOPidx;
    logic [7:0] PIDreceived;
    assign firstSYNC = waiting && (prevStream == 1'b1 && inStream == 1'b0);

    assign finishedOut = isDone;
    assign correct = ~isError;
    assign isAck = PIDreceived == 8'b1101_0010;
    assign isNak = PIDreceived == 8'b0101_1010;

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            waiting <= 1'b1;
            readSYNC <= 1'b0;
            readPID <= 1'b0;
            readEOP <= 1'b0;
            SYNCidx <= 4'b0;
            PIDidx <= 4'b0;
            isDone <= 1'b0;
            isError <= 1'b0;
            PIDreceived <= 8'b0;
        end else begin
            if (waiting) begin
                if (startOut & prevStream == 1'b1 && inStream == 1'b0) begin
                    waiting <= 1'b0;
                    readSYNC <= 1'b1;

                    SYNCidx <= 4'd1;
                    NRZI_stream <= inStream;
                end
            end else if (readSYNC) begin
                NRZI_stream <= inStream;
                SYNCidx <= SYNCidx + 1;
                if (SYNCidx < 4'd7) begin
                    if (NRZI_out != 1'b0) begin
                        isError <= 1'b1;
                    end
                end
                if (SYNCidx == 4'd7) begin
                    if (NRZI_out != 1'b1) begin
                        isError <= 1'b1;
                    end
                    readSYNC <= 1'b0;
                    readPID <= 1'b1;
                    PIDidx <= 4'b0;
                end
            end else if (readPID) begin
                NRZI_stream <= inStream;
                PIDidx <= PIDidx + 1;
                PIDreceived[4'd7 - PIDidx] <= NRZI_out;
                if (PIDidx == 4'd7) begin
                    readPID <= 1'b0;
                    readEOP <= 1'b1;
                end
            end else if (readEOP) begin
                EOPidx <= EOPidx + 4'b1;
                if (~isAck & ~isNak) begin
                    isError <= 1'b1;
                end
                 if (EOPidx <= 4'b1) begin
                    if (inStream != 1'b0) begin
                        isError <= 1'b1;
                    end
                end else if (EOPidx == 4'd2) begin
                    if (~isSE0) begin
                        isError <= 1'b1;
                    end
                    readEOP <= 1'b0;
                    isDone <= 1'b1;
                end
            end else if (isDone) begin
                waiting <= 1'b1;
            end
        end
    end

    // Timeout logic
    logic [8:0] timeoutCnt;
    logic is_TO;

    assign is_TO = timeoutCnt == 9'd255;

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            timeoutCnt <= '0;
        end else begin
            if (waiting) begin
                timeoutCnt <= timeoutCnt + 9'b1;
                if (is_TO) begin
                    timeoutCnt <= '0;
                end
            end else begin
                timeoutCnt <= '0;
            end
        end
    end
endmodule : AckNakInPacket


module ReadWriteFSM (
    USBWires wires,
    input logic clock, reset_n,
    input logic startRead, startWrite,
    input logic finished, error,
    input logic [15:0] mempage,
    input logic [63:0] dataSend,

    output logic in_start, out_start,
    output logic [63:0] dataOut,
    output logic read_write_FSM_done,
    output logic isValueReadCorrect,
    output logic success,
    output logic [63:0] dataReceived
);

    logic waiting, address, buffer, reading, writing, is_done, is_error;

    // Output from the FSM
    assign read_write_FSM_done = is_done;
    assign isValueReadCorrect = ~is_error;
    assign success = ~is_error;
    assign {wires.DP, wires.DM} = buffer ? BS_SE0 : BS_NC;

    always_comb begin
        in_start = 1'b0; out_start = 1'b0;
        dataOut = 64'd0;
        if (address) begin
            out_start = 1'b1;
            dataOut = {48'b0, mempage};
            // in_start or out_start should be 1'b1
        end else if (waiting) begin
            if (startRead | startWrite) begin
                out_start = 1'b1;
                dataOut = {48'b0, mempage};
            end
        end else if (writing) begin
            out_start = 1'b1;
            dataOut = dataSend;
        end else if (reading) begin
            in_start = 1'b1;
        end
    end

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            waiting <= 1'b1;
            address <= 1'b0;
            buffer <= 1'b0;
            reading <= 1'b0;
            writing <= 1'b0;
            is_done <= 1'b0;
            is_error <= 1'b0;
        end else begin
            // READING
            if (reading) begin
                if (finished) begin
                    reading <= 1'b0;
                    is_done <= 1'b1;
                    is_error <= error;
                end
            end
            // WRITING
            else if (writing) begin
                if (finished) begin
                    writing <= 1'b0;
                    is_done <= 1'b1;
                    is_error <= error;
                end
            end
            // DONE and ERROR
            else if (is_done) begin
                is_done <= 1'b0;
                is_error <= 1'b1;
            end
            // ADDRESS
            else if (address & finished) begin
                address <= 1'b0;
                if (startRead) begin
                    reading <= 1'b1;
                end else if (startWrite) begin
                    writing <= 1'b1;
                end
            end
            // BUFFER
            else if (buffer) begin
                if (startRead) begin
                    buffer <= 1'b0;
                    reading <= 1'b1;
                end else if (startWrite) begin
                    buffer <= 1'b0;
                    writing <= 1'b1;
                end
            end
            // WAITING
            else if (waiting) begin
                if (startRead | startWrite) begin
                    waiting <= 1'b0;
                    address <= 1'b1;
                    is_error <= 1'b0;
                end
            end
        end
    end
endmodule : ReadWriteFSM



module ProtocolHandler (
    USBWires wires,
    input logic clock, reset_n,
    input logic in_start, out_start,
    input logic finishedIn, finishedOut,
    input logic finishedAck, finishedNack,
    input logic finishedAckIn, finishedNackIn,
    input logic finishedData, finishedDataIn,
    input logic errorDataIn,

    output logic startOut, startIn,
    output logic startAck, startNack,
    output logic startAckIn, startNackIn,
    output logic startData, startDataIn,
    output logic finished, error
);

    logic [3:0] retries;

    enum logic [3:0] {WAIT, SEND_IN, SEND_OUT, REC_DATA0,
                      SEND_DATA0, SEND_ACK, SEND_NAK,
                      REC_ACK, DONE, ERROR} currState, nextState;

    assign startIn = (currState == SEND_IN) & ~finishedIn;
    assign startDataIn = (currState == REC_DATA0) & ~finishedDataIn;
    assign startAck = (currState == SEND_ACK) & ~finishedAck;
    assign startNack = (currState == SEND_NAK) & ~finishedNack;
    assign startOut = (currState == SEND_OUT) & ~finishedOut;
    assign startData = (currState == SEND_DATA0) & ~finishedData;
    assign startAckIn = (currState == REC_ACK) & ~(finishedAckIn | finishedNackIn);
    assign startNackIn = (currState == REC_ACK) & ~(finishedNackIn | finishedAckIn);

    // State Transitions
    always_comb begin
        nextState = currState;
        case (currState)
            WAIT: begin
                if (in_start) begin
                    nextState = SEND_IN;
                end else if (out_start) begin
                    nextState = SEND_OUT;
                end
            end
            SEND_IN: begin
                if (finishedIn) begin
                    nextState = REC_DATA0;
                end
            end
            REC_DATA0: begin
                if (finishedDataIn) begin
                    if (errorDataIn) begin
                        nextState = SEND_NAK;
                    end else begin
                        nextState = SEND_ACK;
                    end
                end
            end
            SEND_ACK: begin
                if (finishedAck) begin
                    nextState = DONE;
                end
            end
            SEND_NAK: begin
                if (finishedNack) begin
                    if(retries < 8) begin
                        nextState = REC_DATA0;
                    end else begin
                        nextState = ERROR;
                    end
                end
            end
            SEND_OUT: begin
                if (finishedOut) begin
                    nextState = SEND_DATA0;
                end
            end
            SEND_DATA0: begin
                if (finishedData) begin
                    nextState = REC_ACK;
                end
            end
            REC_ACK: begin
                if (finishedNackIn) begin
                    if (retries < 8) begin
                        nextState = SEND_DATA0;
                    end else begin
                        nextState = ERROR;
                    end
                end else if (finishedAckIn) begin
                    nextState = DONE;
                end
            end
            DONE: begin
                nextState = WAIT;
            end
            ERROR: begin
                nextState = WAIT;
            end
        endcase
    end

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            currState <= WAIT;
        end else begin
            currState <= nextState;
            if (currState == SEND_NAK) begin
                if (nextState == REC_DATA0)
                    retries += 4'd1;
                else if (nextState == ERROR)
                    retries = 4'd0;
            end else if (currState == REC_ACK) begin
                if (nextState == SEND_DATA0)
                    retries += 4'd1;
                else if (nextState == ERROR || nextState == DONE)
                    retries = 4'd0;
            end else if (currState == REC_DATA0 && nextState == SEND_ACK) begin
                retries = 4'd0;
            end
        end
    end
endmodule : ProtocolHandler


module USBHost (
  USBWires wires,
  input logic clock, reset_n
);
logic startOut, startIn, startDataIn, startAck, startNack, startAckIn, startNackIn;
logic finishedOut, finishedIn, finishedDataIn, finishedAck, finishedNack, finishedAckIn, finishedNackIn;

logic startData, finishedData;
logic [15:0] mempageAt;
logic [63:0] dataSend, dataDetected, dataOut, dataReceived;

InOutPacket #(0) OUT (.startOut(startOut), .clock(clock), .reset_n(reset_n),
                      .finishedOut(finishedOut), .wires(wires));
InOutPacket #(1) IN  (.startOut(startIn), .clock(clock), .reset_n(reset_n),
                      .finishedOut(finishedIn), .wires(wires));
AckNackPacket #(1) ACK (.startOut(startAck), .clock(clock), .reset_n(reset_n),
                        .finishedOut(finishedAck), .wires(wires));
AckNackPacket #(0) NACK (.startOut(startNack), .clock(clock),
                         .reset_n(reset_n),
                         .finishedOut(finishedNack), .wires(wires));
DataPacket Test (.startOut(startData), .data(dataSend), .clock(clock),
                 .reset_n(reset_n),
                 .finishedOut(finishedData), .wires(wires));
DataInPacket Test2 (.wires(wires), .start(startDataIn), .clock(clock),
                    .reset_n(reset_n), .finished(finishedDataIn),
                    .data(dataDetected));

AckNackInPacket #(0) ACKTest (.wires(wires), .start(startAckIn), .clock(clock),
                              .reset_n(reset_n), .finished(finishedAckIn));
AckNackInPacket #(1) ACKTest1 (.wires(wires), .start(startNackIn), .clock(clock),
                              .reset_n(reset_n), .finished(finishedNackIn));

logic startRead, startWrite, proHandFinished, proHandError;
logic in_start, out_start;
logic read_write_FSM_done, isValueReadCorrect, writeSuccess;
ReadWriteFSM rwFSM (.wires, .clock, .reset_n, .startRead, .startWrite,
                    .finished(proHandFinished), .error(proHandError),
                    .in_start, .out_start,
                    .mempage(mempageAt), .dataSend, .dataOut, .dataReceived,
                    .read_write_FSM_done, .isValueReadCorrect,
                    .success(writeSuccess));
ProtocolHandler proHand(.wires, .clock, .reset_n, .in_start, .out_start,
                        .finishedIn, .finishedOut, .finishedAck, .finishedNack,
                        .finishedAckIn, .finishedNackIn,
                        // TODO: errorDataIn
                        .finishedData, .finishedDataIn, .errorDataIn(1'b0),
                        .startOut, .startIn, .startAck, .startNack,
                        .startAckIn, .startNackIn, .startData,
                        .startDataIn, .finished(proHandFinished),
                        .error(proHandError));

task prelabRequest();
  start = 1;
  dataSend = 64'h40aa_11b7_682d_f6d8;

  while (!finished) begin
    @(posedge clock);
  end
endtask : prelabRequest

task readData
// Host sends mempage to thumb drive using a READ (OUT->DATA0->IN->DATA0)
// transaction, and then receives data from it. This task should return both the
// data and the transaction status, successful or unsuccessful, to the caller.
( input logic [15:0] mempage, // Page to write
  output logic [63:0] data, // Vector of bytes to write
  output logic success);

    mempageAt <= mempage;
    startRead <= 1'b1;

    wait (read_write_FSM_done);

    startRead <= 1'b0;
    data <= dataReceived;
    success <= isValueReadCorrect;
endtask : readData

task writeData
// Host sends mempage to thumb drive using a WRITE (OUT->DATA0->OUT->DATA0)
// transaction, and then sends data to it. This task should return the
// transaction status, successful or unsuccessful, to the caller.
( input logic [15:0] mempage, // Page to write
  input logic [63:0] data, // Vector of bytes to write
  output logic success);

    mempageAt <= mempage;
    dataSend <= data;
    startWrite <= 1'b1;

    wait (read_write_FSM_done);

    startWrite <= 1'b0;
    success <= writeSuccess;
endtask : writeData

// Testing for NRZI
// logic stream, ready, out, check;
// NRZI nrzi(.stream, .ready, .clock, .reset_n, .out);
// NRZIDecoder(.stream(out), .ready, .clock, .reset_n, .out(check));
//
// task testNRZI();
//     ready <= 1'b1;
//     @(posedge clock);
//     repeat(50) begin
//         stream <= $urandom_range(1, 0);
//         assert(stream == check) else $error("not equal");
//         @(posedge clock);
//     end
// endtask : testNRZI

// Testing for BitUnstuffer + BitStreamDecoder
// logic stream, ready, unstuffReady, out, hold, fin, bsFin;
// logic [31:0] parallelOut, parallelIn;
// BitStuffer #32 bs(.clock, .reset_n, .out(stream), .ready,
//                   .parallelIn, .finished(bsFin));
// BitUnstuffer bu(.clock, .reset_n, .stream, .ready(unstuffReady), .out, .hold);
// BitStreamDecoder #32 bsd(.clock, .reset_n, .out, .ready(unstuffReady),
//                          .stream(out), .out(parallelOut), .hold,
//                          .finished(fin));
//
// task testUnstuffing();
//     ready <= 1'b1;
//     @(posedge clock);
//     unstuffReady <= 1'b1;
//
//     while(!fin)
//         @(posedge clock);
//     @(posedge clock);
//     assert(parallelIn == parallelOut)
//         else $error("%b != %b", parallelIn, parallelOut);
//     @(posedge clock);
// endtask: testUnstuffing

// Testing for CRC5 checker
// logic stream, ready, done, correct, crc5done, crc5ready;
// logic [10:0] parallelIn;
// logic [4:0] crc5;
// CRC5 crc(.parallelIn, .ready(crc5ready), .done(crc5done), .out(crc5),
//          .clock, .reset_n);
// CRC5Checker #16 crccheck(.clock, .reset_n, .stream, .ready, .done,
//                          .correct);
//
// task testCRC5Checker();
//     crc5ready <= 1'b1;
//     parallelIn <= 11'b00001000111;
//     while(!crc5done)
//         @(posedge clock);
//     ready <= 1'b1;
//     @(posedge clock);
//     for (int i = 10; i >= 0; i=i-1) begin
//         stream <= parallelIn[i];
//         @(posedge clock);
//     end
//     for (int i = 4; i >= 0; i=i-1) begin
//         stream <= ~crc5[i];
//         @(posedge clock);
//     end
//     @(posedge clock);
//     @(posedge clock);
//     assert(done) else $error("done not asserted");
//     assert(correct) else $error("correct not asserted: %b", crc5);
// endtask: testCRC5Checker

// Testing for CRC16 checker
// logic stream, ready, done, correct, crc16done, crc16ready;
// logic [63:0] parallelIn;
// logic [15:0] crc16;
// CRC16 crc(.parallelIn, .ready(crc16ready), .done(crc16done), .out(crc16),
//          .clock, .reset_n);
// CRC16Checker #80 crccheck(.clock, .reset_n, .stream, .ready, .done,
//                          .correct);
//
// task testCRC16Checker();
//     crc16ready <= 1'b1;
//     parallelIn <= 64'h40aa_11b7_682d_f6d8;
//     while(!crc16done)
//         @(posedge clock);
//     ready <= 1'b1;
//     @(posedge clock);
//     ready <= 1'b0;
//     for (int i = 63; i >= 0; i=i-1) begin
//         stream <= parallelIn[i];
//         @(posedge clock);
//     end
//     for (int i = 15; i >= 0; i=i-1) begin
//         stream <= ~crc16[i];
//         @(posedge clock);
//     end
//     @(posedge clock);
//     @(posedge clock);
//     assert(done) else $error("done not asserted");
//     assert(correct) else $error("correct not asserted: %x", crc16);
// endtask: testCRC16Checker
  
endmodule : USBHost







