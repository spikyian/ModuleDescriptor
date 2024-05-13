#!/bin/sh

# Generate all descriptor files that have generators.

gen_dir=`dirname $0`
wip_dir=`dirname $gen_dir`/work_in_progress

$gen_dir/generate_CANLEVER.sh > CANLEVER-0D20-1a.json

$gen_dir/generate_CANMIO-SVO.sh > CANMIO-SVO-A532-4s.json

$gen_dir/generate_CANMIO.sh > $wip_dir/CANMIO-A520-3a.json
$gen_dir/generate_CANMIO.sh > $wip_dir/CANMIO-A520-3c.json
$gen_dir/generate_CANMIO-A520-3d.sh > $wip_dir/CANMIO-A520-3d.json
$gen_dir/generate_CANMIO-A520-3e.sh > $wip_dir/CANMIO-A520-3e.json

$gen_dir/generate_CANXIO-A540-3e.sh > $wip_dir/CANXIO-A540-3e.json

$gen_dir/generate_CANPAN.sh > $wip_dir/CANPAN-A51D-1y.json