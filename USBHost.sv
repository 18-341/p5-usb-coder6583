`include "USBPkg.pkg"

// Wrapper for USB bus states. Notice that enum Z can only be driven, not read
typedef enum logic [1:0]
  {BS_J = 2'b10, BS_K = 2'b01, BS_SE0 = 2'b00, BS_SE1 = 2'b11, BS_NC = 2'bzz}
  bus_state_t;


module NRZI (
  input logic stream,
  input logic clock, reset_n,
  output logic out,
);
  always_ff @(posedge clock, negedge reset_n) begin
    if (reset_n == 0)
      out <= 1;
    else begin 
      if (stream) begin 
        out <= ~out;
      end
      else begin 
        out <= out;
      end
    end
  end
endmodule NRZI;

module BitStuffer (
  input logic stream;
  input logic clock, reset_n,
  output logic pause,
  output logic out
)
  logic [3:0] count;
  always_ff @(posedge clock, negedge reset_n) begin
    if (reset_n == 0)
      count <= 0;
    else begin
      if (pause == 1) begin
        out <= 0;
        pause <= 0;
      end
      else begin 
        if (stream == 1) begin 
          count <= count + 1;
        end
        else begin
          count <= 0;
        end
        if (count == 6) begin 
          pause <= 1;
          count <= 0;
        end
        else begin
          out <= steam;
        end
      end
    end
  end
endmodule BitStuffer;


module USBHost (
  USBWires wires,
  input logic clock, reset_n
);

task prelabRequest();
  logic [6:0] device_address;
  logic [3:0] data_endpoint;
  device_address = DEVICE_ADDR;
  data_endpoint = DATA_ENDP;
  

  logic [23:0] packet;
  packet = 24'b1101_1111_1000_1111_1111_0011;
  
  
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
