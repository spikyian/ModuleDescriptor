#!/bin/sh

# Generate all descriptor files that have generators.

gen_dir=`dirname $0`
wip_dir=`dirname $gen_dir`/work_in_progress

$gen_dir/generate_CANLEVER.sh > $wip_dir/CANLEVER-0D20-1a.json
echo "Generated $wip_dir/CANLEVER-0D20-1a.json"

$gen_dir/generate_CANMIO-SVO.sh > $wip_dir/CANMIO-SVO-A532-4s.json
echo "Generated $wip_dir/CANMIO-SVO-A532-4s.json"

$gen_dir/generate_CANMIO.sh 3a > $wip_dir/CANMIO-A520-3a.json
echo "Generated $wip_dir/CANMIO-A520-3a.json"
$gen_dir/generate_CANMIO.sh 3c > $wip_dir/CANMIO-A520-3c.json
echo "Generated $wip_dir/CANMIO-A520-3c.json"
$gen_dir/generate_CANMIO.sh 3d > $wip_dir/CANMIO-A520-3d.json
echo "Generated $wip_dir/CANMIO-A520-3d.json"
$gen_dir/generate_CANMIO.sh 3e > $wip_dir/CANMIO-A520-3e.json
echo "Generated $wip_dir/CANMIO-A520-3e.json"
$gen_dir/generate_CANMIO.sh 4a > $wip_dir/CANMIO-A520-4a.json
echo "Generated $wip_dir/CANMIO-A520-4a.json"

$gen_dir/generate_CANMIO.sh XIO 3e > $wip_dir/CANXIO-A540-3e.json
echo "Generated $wip_dir/CANXIO-A540-3e.json"

$gen_dir/generate_CANPAN.sh > $wip_dir/CANPAN-A51D-1y.json
echo "Generated $wip_dir/CANPAN-A51D-1y.json"
