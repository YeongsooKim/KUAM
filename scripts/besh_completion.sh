#!/bin/bash
_kuam_data_processing()
{
    local cur
    cur=${COMP_WORDS[COMP_CWORD]}
    COMPREPLY=( $(compgen -W 'aruco yolo sim rviz' -- $cur ) )
}
complete -F _kuam_data_processing ./launch-data_processing.sh

_kuam_manuever_planning()
{
    local cur
    cur=${COMP_WORDS[COMP_CWORD]}
    COMPREPLY=( $(compgen -W 'camera rviz' -- $cur ) )
}
complete -F _kuam_manuever_planning ./launch-maneuver_planning.sh

_kuam_px4()
{
    local cur
    cur=${COMP_WORDS[COMP_CWORD]}
    COMPREPLY=( $(compgen -W 'sim' -- $cur ) )
}
complete -F _kuam_px4 ./launch-px4.sh