#!/bin/sh

# Generate all descriptor files that have generators.

gen_dir=`dirname $0`
wip_dir=`dirname $gen_dir`/work_in_progress
tmp_file=t.json

function writeIfUpdated()
{
  outFileName=$1
  cat > $tmp_file
  if diff -qw -I'"timestamp" *:' $outFileName $tmp_file 2>/dev/null
  then
    rm $tmp_file
    echo "No changes to $outFileName"
  else
    mv $tmp_file "$outFileName"
    echo "Generated $outFileName"
  fi
}

$gen_dir/generate_CANLEVER.sh | writeIfUpdated $wip_dir/CANLEVER-0D20-1a.json

$gen_dir/generate_CANMIO-SVO.sh | writeIfUpdated $wip_dir/CANMIO-SVO-A532-4s.json

# CANMIO
# Default processor is PIC18F26K80
$gen_dir/generate_CANMIO.sh -v 3a | writeIfUpdated $wip_dir/CANMIO-A520-3a.json
$gen_dir/generate_CANMIO.sh -v 3c | writeIfUpdated $wip_dir/CANMIO-A520-3c.json
$gen_dir/generate_CANMIO.sh -v 3d | writeIfUpdated $wip_dir/CANMIO-A520-3d.json
$gen_dir/generate_CANMIO.sh -v 3e | writeIfUpdated $wip_dir/CANMIO-A520-3e.json
$gen_dir/generate_CANMIO.sh -v 4a | writeIfUpdated $wip_dir/CANMIO-A520-4a.json

# Processor P18F27Q83
$gen_dir/generate_CANMIO.sh -p23 -v 4a | writeIfUpdated $wip_dir/CANMIO-A520-4a--P23.json

# Extended CANMIO
# Default processor PIC18F46K80
$gen_dir/generate_CANMIO.sh -t XIO -v 3e | writeIfUpdated $wip_dir/CANXIO-A540-3e.json
$gen_dir/generate_CANMIO.sh -t XIO -v 4a | writeIfUpdated $wip_dir/CANXIO-A540-4a.json
$gen_dir/generate_CANMIO.sh -t XIO -p22 -v 4a | writeIfUpdated $wip_dir/CANXIO-A540-4a--P21.json

$gen_dir/generate_CANPAN.sh -v 1Y | writeIfUpdated $wip_dir/CANPAN-A51D-1Y.json
$gen_dir/generate_CANPAN.sh -v 4C | writeIfUpdated $wip_dir/CANPAN-A51D-4C.json
