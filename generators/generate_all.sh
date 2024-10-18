#!/bin/sh

# Generate all descriptor files that have generators.

gen_dir=`dirname $0`
wip_dir=`dirname $gen_dir`/work_in_progress

$gen_dir/generate_CANLEVER.sh > $wip_dir/CANLEVER-0D20-1a.json
echo "Generated $wip_dir/CANLEVER-0D20-1a.json"

$gen_dir/generate_CANMIO-SVO.sh > $wip_dir/CANMIO-SVO-A532-4s.json
echo "Generated $wip_dir/CANMIO-SVO-A532-4s.json"

# CANMIO
# Default processor is PIC18F26K80
$gen_dir/generate_CANMIO.sh -v 3a > $wip_dir/CANMIO-A520-3a.json
echo "Generated $wip_dir/CANMIO-A520-3a.json"
$gen_dir/generate_CANMIO.sh -v 3c > $wip_dir/CANMIO-A520-3c.json
echo "Generated $wip_dir/CANMIO-A520-3c.json"
$gen_dir/generate_CANMIO.sh -v 3d > $wip_dir/CANMIO-A520-3d.json
echo "Generated $wip_dir/CANMIO-A520-3d.json"
$gen_dir/generate_CANMIO.sh -v 3e > $wip_dir/CANMIO-A520-3e.json
echo "Generated $wip_dir/CANMIO-A520-3e.json"
$gen_dir/generate_CANMIO.sh -v 4a > $wip_dir/CANMIO-A520-4a.json
echo "Generated $wip_dir/CANMIO-A520-4a.json"

# Processor P18F27Q83
$gen_dir/generate_CANMIO.sh -p23 -v 4a > $wip_dir/CANMIO-A520-4a--P23.json
echo "Generated $wip_dir/CANMIO-A520-4a--P23.json"

# Extended CANMIO
# Default processor PIC18F46K80
$gen_dir/generate_CANMIO.sh -t XIO -v 3e > $wip_dir/CANXIO-A540-3e.json
echo "Generated $wip_dir/CANXIO-A540-3e.json"
$gen_dir/generate_CANMIO.sh -t XIO -v 4a > $wip_dir/CANXIO-A540-4a.json
echo "Generated $wip_dir/CANXIO-A540-4a.json"
$gen_dir/generate_CANMIO.sh -t XIO -p22 -v 4a > $wip_dir/CANXIO-A540-4a--P21.json
echo "Generated $wip_dir/CANXIO-A540-4a--P22.json"

$gen_dir/generate_CANPAN.sh > $wip_dir/CANPAN-A51D-1y.json
echo "Generated $wip_dir/CANPAN-A51D-1y.json"
