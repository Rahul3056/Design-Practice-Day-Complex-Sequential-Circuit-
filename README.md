
### Design Practice Day (Complex Sequential Circuit)

**Concept Overview**  
Complex sequential circuits are digital systems whose outputs depend not only on the current inputs but also on the past history of inputs, i.e., stored states. These designs typically include multiple flip-flops, counters, control logic, and sometimes state machines. They're used in controllers, traffic light systems, processors, and more.

Today’s focus is to design a **sequence detector** — a fundamental yet practical complex sequential system.

---

### Design Example: Sequence Detector for 1011 (Overlapping Allowed)

**Objective**  
Detect the binary sequence `1011` from a serial input stream. For every detection of the sequence, raise an output `1` for one clock cycle. Overlapping sequences should be detected (e.g., in `1101011`, two sequences should be detected).

---

### State Diagram  
The design follows a Moore or Mealy FSM model. Here, we use the **Mealy model** (output depends on both state and input).

States:
- `S0`: Initial state  
- `S1`: Detected `1`  
- `S2`: Detected `10`  
- `S3`: Detected `101`

Transition to final state with `1` on input: output `1`.

---

### State Encoding

| State | Binary Code |
|-------|-------------|
| S0    | 2'b00       |
| S1    | 2'b01       |
| S2    | 2'b10       |
| S3    | 2'b11       |

---

### Truth Table / State Table

| Current State | Input | Next State | Output |
|---------------|-------|------------|--------|
| S0            | 0     | S0         | 0      |
| S0            | 1     | S1         | 0      |
| S1            | 0     | S2         | 0      |
| S1            | 1     | S1         | 0      |
| S2            | 0     | S0         | 0      |
| S2            | 1     | S3         | 0      |
| S3            | 0     | S2         | 0      |
| S3            | 1     | S1         | 1      |

---

### Verilog Implementation

```verilog
module sequence_detector (
    input clk,
    input rst,
    input in_bit,
    output reg detected
);
    typedef enum logic [1:0] {
        S0 = 2'b00,
        S1 = 2'b01,
        S2 = 2'b10,
        S3 = 2'b11
    } state_t;

    state_t state, next_state;

    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            state <= S0;
        else
            state <= next_state;
    end

    always_comb begin
        next_state = state;
        detected = 0;
        case (state)
            S0: next_state = (in_bit) ? S1 : S0;
            S1: next_state = (in_bit) ? S1 : S2;
            S2: next_state = (in_bit) ? S3 : S0;
            S3: begin
                next_state = (in_bit) ? S1 : S2;
                detected = (in_bit) ? 1 : 0;
            end
        endcase
    end
endmodule
```

---

### Testbench

```verilog
module tb_sequence_detector;
    reg clk, rst, in_bit;
    wire detected;

    sequence_detector uut (
        .clk(clk),
        .rst(rst),
        .in_bit(in_bit),
        .detected(detected)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        $monitor("Time=%0t | in_bit=%b | detected=%b", $time, in_bit, detected);
        rst = 1; in_bit = 0; #10;
        rst = 0;

        in_bit = 1; #10;
        in_bit = 0; #10;
        in_bit = 1; #10;
        in_bit = 1; #10;
        in_bit = 0; #10;
        in_bit = 1; #10;
        in_bit = 0; #10;
        in_bit = 1; #10;
        in_bit = 1; #10;

        $finish;
    end
endmodule
```

---

### Simulation Output (Expected)

```
Time=10 | in_bit=1 | detected=0
Time=20 | in_bit=0 | detected=0
Time=30 | in_bit=1 | detected=0
Time=40 | in_bit=1 | detected=1
Time=50 | in_bit=0 | detected=0
Time=60 | in_bit=1 | detected=0
Time=70 | in_bit=0 | detected=0
Time=80 | in_bit=1 | detected=0
Time=90 | in_bit=1 | detected=1

### Real-World Applications

- Pattern detection in data streams
- Control sequence monitoring in processors
- Digital lock combinations
- Security and ID validation logic
