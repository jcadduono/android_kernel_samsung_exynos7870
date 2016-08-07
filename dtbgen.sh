#!/bin/bash
# simple bash script for generating dtb image

# root directory of universal7870 kernel git repo (default is this script's location)
RDIR=$(pwd)

# directory containing cross-compile arm64 toolchain
TOOLCHAIN=$HOME/build/toolchain/gcc-linaro-4.9-2016.02-x86_64_aarch64-linux-gnu

# device dependant variables
PAGE_SIZE=2048
DTB_PADDING=0

export ARCH=arm64
export CROSS_COMPILE=$TOOLCHAIN/bin/aarch64-linux-gnu-

BDIR=$RDIR/build
OUTDIR=$BDIR/arch/$ARCH/boot
DTSDIR=$RDIR/arch/$ARCH/boot/dts
DTBDIR=$OUTDIR/dtb
DTCTOOL=$BDIR/scripts/dtc/dtc
INCDIR=$RDIR/include

ABORT()
{
	[ "$1" ] && echo "Error: $*"
	exit 1
}

[ -x "$DTCTOOL" ] ||
ABORT "You need to run ./build.sh first!"

[ -x "${CROSS_COMPILE}gcc" ] ||
ABORT "Unable to find gcc cross-compiler at location: ${CROSS_COMPILE}gcc"

[ "$1" ] && DEVICE=$1
[ "$2" ] && VARIANT=$2

case $DEVICE in
gtaxlwifi)
	case $VARIANT in
	xx|eur)
		DTSFILES="exynos7870-gtaxlwifi_eur_open_00 exynos7870-gtaxlwifi_eur_open_04"
		;;
	*) ABORT "Unknown variant of $DEVICE: $VARIANT" ;;
	esac
	DTBH_PLATFORM_CODE=0x50a6
	DTBH_SUBTYPE_CODE=0x217584da
	;;
gtaxllte)
	case $VARIANT in
	xx|eur)
		DTSFILES="exynos7870-gtaxllte_eur_open_00 exynos7870-gtaxllte_eur_open_01
			exynos7870-gtaxllte_eur_open_04 exynos7870-gtaxllte_eur_open_05"
		;;
	*) ABORT "Unknown variant of $DEVICE: $VARIANT" ;;
	esac
	DTBH_PLATFORM_CODE=0x50a6
	DTBH_SUBTYPE_CODE=0x217584da
	;;
gtanotexlwifi)
	case $VARIANT in
	kor|ktt|lgt|skt)
		DTSFILES="exynos7870-gtanotexlwifi_kor_open_00 exynos7870-gtanotexlwifi_kor_open_02"
		;;
	*) ABORT "Unknown variant of $DEVICE: $VARIANT" ;;
	esac
	DTBH_PLATFORM_CODE=0x50a6
	DTBH_SUBTYPE_CODE=0x217584da
	;;
gtanotexllte)
	case $VARIANT in
	kor|ktt|lgt|skt)
		DTSFILES="exynos7870-gtanotexllte_kor_open_00 exynos7870-gtanotexllte_kor_open_02"
		;;
	*) ABORT "Unknown variant of $DEVICE: $VARIANT" ;;
	esac
	DTBH_PLATFORM_CODE=0x50a6
	DTBH_SUBTYPE_CODE=0x217584da
	;;
matisse10wifi)
	case $VARIANT in
	kor|ktt|lgt|skt)
		DTSFILES="exynos7870-matisse10wifi_kor_open_00 exynos7870-matisse10wifi_kor_open_01
			exynos7870-matisse10wifi_kor_open_02 exynos7870-matisse10wifi_kor_open_03"
		;;
	*) ABORT "Unknown variant of $DEVICE: $VARIANT" ;;
	esac
	DTBH_PLATFORM_CODE=0x50a6
	DTBH_SUBTYPE_CODE=0x217584da
	;;
j7xelte)
	case $VARIANT in
	xx|eur)
		DTSFILES="exynos7870-j7xelte_eur_open_00 exynos7870-j7xelte_eur_open_01
			exynos7870-j7xelte_eur_open_02 exynos7870-j7xelte_eur_open_03
			exynos7870-j7xelte_eur_open_04"
		;;
	kor|ktt|lgt|skt)
		DTSFILES="exynos7870-j7xelte_kor_03 exynos7870-j7xelte_kor_04"
		;;
	*) ABORT "Unknown variant of $DEVICE: $VARIANT" ;;
	esac
	DTBH_PLATFORM_CODE=0x50a6
	DTBH_SUBTYPE_CODE=0x217584da
	;;
degas2wifi)
	case $VARIANT in
	xx|eur)
		DTSFILES="exynos7870-degas2wifi_eur_bmw_00 exynos7870-degas2wifi_eur_bmw_01"
		;;
	*) ABORT "Unknown variant of $DEVICE: $VARIANT" ;;
	esac
	DTBH_PLATFORM_CODE=0x50a6
	DTBH_SUBTYPE_CODE=0x217584da
	;;
a7maxlte)
	case $VARIANT in
	xx|eur)
		DTSFILES="exynos7870-a7maxlte_eur_open_00"
		;;
	*) ABORT "Unknown variant of $DEVICE: $VARIANT" ;;
	esac
	DTBH_PLATFORM_CODE=0x50a6
	DTBH_SUBTYPE_CODE=0x217584da
	;;
*) ABORT "Unknown device: $DEVICE" ;;
esac

mkdir -p "$OUTDIR" "$DTBDIR"

cd "$DTBDIR" || ABORT "Unable to cd to $DTBDIR!"

rm -f ./*

echo "Processing dts files..."

for dts in $DTSFILES; do
	echo "=> Processing: ${dts}.dts"
	"${CROSS_COMPILE}cpp" -nostdinc -undef -x assembler-with-cpp -I "$INCDIR" "$DTSDIR/${dts}.dts" > "${dts}.dts"
	echo "=> Generating: ${dts}.dtb"
	$DTCTOOL -p $DTB_PADDING -i "$DTSDIR" -O dtb -o "${dts}.dtb" "${dts}.dts"
done

echo "Generating dtb.img..."
"$RDIR/scripts/dtbTool/dtbTool" -o "$OUTDIR/dtb.img" -d "$DTBDIR/" -s $PAGE_SIZE --platform $DTBH_PLATFORM_CODE --subtype $DTBH_SUBTYPE_CODE || exit 1

echo "Done."
