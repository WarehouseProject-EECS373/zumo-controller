# Functions for examining and manipulating the stack in gdb.

# Script constants.
set $one_kb        = 1024.0
set $safety_margin = 16

# Raspbian Linux stack parameters.
set $stack_start   = 0x20000000
set $stack_end     = 0x20000000 + 640000
set $stack_size    = $stack_end - $stack_start

define stack_args
    if $argc < 2
        printf "Usage: stack_args <offset|start> <length|end>\n"
    else
        if $arg0 < $stack_start
            # Assume arg0 is a relative offset from start of stack.
            set $offset = (int)$arg0
        else
            # Assume arg0 is an absolute address, so compute its offset.
            set $offset = (int)$arg0 - $stack_start
        end
        
        if $arg1 < $stack_start
            # Assume arg1 is a relative length.
            set $length = (int)$arg1
        else
            # Assume arg1 is an absolute address, so compute its length.
            set $length = (int)$arg1 - $stack_start - $offset
        end
    end
end

document stack_args
Usage: stack_args <offset|start> <length|end>

Set stack region offset and length from arguments.
end

define dump_stack
    if $argc < 2
        printf "Usage: dump_stack <offset|start> <length|end>\n"
    else
        stack_args $arg0 $arg1
        
        set $i = 0
        while $i < $length
            set $addr = $stack_start + $offset + $i
            x/4wx $addr
            set $i = $i + 16
        end
    end
end

document dump_stack
Usage: dump_stack <offset|start> <length|end>

Dumps stack starting at <offset|start> bytes, 4 longwords at a time,
for <length|end> bytes.
end

define clear_stack
    if $argc < 2
        printf "Usage: clear_stack <offset|start> <length|end>\n"
    else
        stack_args $arg0 $arg1
        
        if $stack_start + $offset + $safety_margin >= $sp
            printf "Error: start is in active stack.\n"
        else
            if $stack_start + $offset + $length + safety_margin >= $sp
                printf "Error: end is in active stack.\n"
            else
                set $i = 0
                while $i < $length
                    set $addr = $stack_start + $offset + $i
                    set *((int *) $addr) = 0
                    set $i = $i + 4
                    
                    # Takes a while, so give some feedback.
                    if $i % 10000 == 0
                        printf "Cleared %d\n", $i
                    end
                end
            end
        end
    end
end

document clear_stack
Usage: clear_stack <offset|start> <length|end>

Clears stack starting at <offset|start> bytes, one longword at a time,
for <length|end> bytes.
end

define stack_offset
    if $argc < 1
        printf "Usage: stack_offset <address>\n"
    else
        # Cast to int is needed to set $depth when $arg0 is $sp.
        set $addr   = (int)$arg0
        set $offset = $addr - $stack_start
        set $depth  = $stack_end - $addr
        
        printf "Address    %10d = 0x%08x\n", $addr, $addr
        
        if $addr < $stack_start || $addr >= $stack_end
            printf "Warning: address is not in stack.\n"
        end
        
        printf "Stack size   %6d = 0x%05x = %5.1fKB, 0x%x-0x%x\n", $stack_size, $stack_size, $stack_size / $one_kb, $stack_start, $stack_end
        printf "Stack offset %6d = 0x%05x = %5.1fKB\n", $offset, $offset, $offset / $one_kb
        printf "Stack depth  %6d = 0x%05x = %5.1fKB\n", $depth, $depth, $depth / $one_kb
    end
end

document stack_offset
Usage: stack_offset <address>

Shows stack offset and depth represented by address.
end

define scan_stack
    if $argc < 2
        printf "Usage: scan_stack <offset|start> <length|end>\n"
    else
        stack_args $arg0 $arg1
        
        set $addr = $stack_start + $offset
        set $i    = 0
        while $i < $length && *((int *) $addr) == 0
            set $addr = $stack_start + $offset + $i
            set $i = $i + 4
            
            # Takes a while, so give some feedback.
            if $i % 10000 == 0
                printf "Scanned %d\n", $i
            end
        end

        if *((int *) $addr) != 0
            if $addr < $sp
                set $offset = $sp - $addr
                printf "Found data %d bytes deeper than current stack frame (0x%x).\n", $offset, $sp
            else
                printf "Stack is clear up to current stack frame (0x%x), it is deepest stack usage.\n", $sp
            end
            
            stack_offset $addr
            dump_stack $addr-$stack_start 64
        else
            printf "Stack is clear in requested range.\n"
        end
    end
end

document scan_stack
Usage: scan_stack <offset|start> <length|end>

Scans stack for non-zero contents starting at <offset|start> bytes, one
longword at a time, for <length|end> bytes.
end

define stack_walk
    set $first_sp = $sp
    set $last_sp  = $sp
    set $total    = 0
    frame
    printf "Top stack frame 0x%08x\n\n", $last_sp
    
    # Loop will error out gracefully when there are no more frames.
    while 1
        up
        set $delta   = $sp - $last_sp
        set $total   = $total + $delta
        printf "Last stack frame 0x%08x, current 0x%08x, size of last %4d = 0x%03x, total deeper %6d = 0x%05x = %5.1fKB\n\n", $last_sp, $sp, $delta, $delta, $total, $total, $total / $one_kb
        set $last_sp = $sp
    end
end

document stack_walk
Usage: stack_walk

Walks stack frames upward from currently selected frame and computes
incremental and cumulative size of frames, so that stack consumption
can be attributed to specific functions.

Use "f 0" to select deepest frame of call stack, or "f <n>" to select
frame <n> higher up in stack.
end