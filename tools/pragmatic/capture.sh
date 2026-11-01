#!/system/bin/sh
# Two endpoints only; run as root. No wakelock or screen-state manipulation.
# Usage: sh capture.sh [seconds=60] [settle_seconds=30] [output_directory]
# DIAG=0 collects a runtime-disabled baseline without changing the debugfs controls.
set -eu
umask 077
seconds=${1:-60}
settle=${2:-30}
out=${3:-/data/local/tmp/pragmatic-$(date +%s)}
diag=/sys/kernel/debug/pragmatic_diag
owned=0
case "$seconds:$settle" in *[!0-9:]*|:*|*:) echo 'Durations must be integers' >&2; exit 1;; esac
[ "$(id -u)" = 0 ] || { echo 'Run as root' >&2; exit 1; }
[ -r "$diag/stats" ] || { echo 'Mount debugfs and check diagnostic support' >&2; exit 1; }
mkdir "$out"
cleanup() {
	if [ "$owned" = 1 ]; then echo 0 > "$diag/enabled"; fi
}
trap cleanup EXIT
trap 'exit 1' HUP INT TERM
sleep "$settle"
# Discover workers before enabling, avoiding a /proc scan inside the interval.
: > "$out/workers"
for f in /proc/[0-9]*/comm; do
	name=$(cat "$f" 2>/dev/null) || continue
	case "$name" in pragmatic:0|pragmatic:4)
		pid=${f#/proc/}; pid=${pid%/comm}
		echo "$pid $name" >> "$out/workers";;
	esac
done
if [ "$(cat "$diag/enabled")" != 0 ]; then
	echo 'Diagnostics already enabled; refusing to disturb another collection' >&2
	exit 1
fi
if [ "${DIAG:-1}" != 0 ]; then
	echo 1 > "$diag/reset"
	owned=1
	echo 1 > "$diag/enabled"
fi
snapshot() {
	dir=$out/$1
	mkdir "$dir"
	date > "$dir/date"
	cat /proc/uptime > "$dir/uptime"
	while read -r pid name; do
		if [ "$(cat /proc/"$pid"/comm 2>/dev/null || :)" = "$name" ]; then
			cat /proc/"$pid"/schedstat > "$dir/$name.schedstat" || :
		else
			echo 'Worker disappeared/restarted; discard this runtime delta' > "$dir/$name.schedstat"
		fi
	done < "$out/workers"
	for cpu in 0 4; do
		base=/sys/devices/system/cpu/cpu$cpu/cpufreq
		for item in stats/total_trans stats/time_in_state stats/trans_table scaling_min_freq scaling_max_freq scaling_governor; do
			if [ -r "$base/$item" ]; then
				cat "$base/$item" > "$dir/cpu$cpu.${item##*/}" || :
			fi
		done
	done
	cat /proc/interrupts > "$dir/interrupts"
	for qos in cluster0_freq_min cluster0_freq_max cluster1_freq_min cluster1_freq_max; do
		file=/sys/kernel/debug/pm_qos/$qos
		if [ -r "$file" ]; then cat "$file" > "$dir/qos.$qos" || :; fi
	done
	cat "$diag/stats" > "$dir/stats"
	cat /proc/uptime > "$dir/uptime_end"
}
snapshot before
sleep "$seconds"
snapshot after
cleanup
owned=0
echo "Saved $out (elapsed time is in uptime; sleep can span suspend)"
