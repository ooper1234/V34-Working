#!/bin/sh
# The scrambler polynomial is the one thing V.34 and V.90 do differently for the
# same physical direction: V.34's receiver descrambles what the modem that
# placed the call sent with GPC, and V.90 6.5 names GPA. Both were tried before
# the receiver tracked the signal at all, which made the results meaningless.
# Redone now that it does.
for cfg in "V90_UP_SCRAMBLER=call" "DUMMY=1" "V90_UP_SCRAMBLER=call V90_UP_TRELLIS=64"; do
    /home/cooper/v90bench/sweep-config.sh "$cfg" 8 || true
done
echo "=== scrambler redone ===" >> /tmp/softmodem/sweep.log
