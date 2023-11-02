if [ -z "$1" ]
  then
    echo "Error: missing argument"
    echo "Syntax: $0 <verilog-file>"
    exit
fi

iverilog -DBENCH -DBOARD_FREQ=10 bench_iverilog.v $1 && ./a.out
