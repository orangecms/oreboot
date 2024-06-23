use crate::util::{
    compile_board_dt, dist_dir, find_binutils_prefix_or_fail, get_cargo_cmd_in, objcopy,
    project_root,
};
use crate::{layout_flash, Cli, Commands, Env, Memory};
use fdt::Fdt;
use log::{error, info, trace, warn};
use std::{
    fs::{self, copy, File},
    path::Path,
    process,
};

extern crate layoutflash;
use layoutflash::areas::{create_areas, Area};

use super::visionfive2_hdr::{spl_create_hdr, HEADER_SIZE};

// The real SRAM size is stated to be 2M, but the mask ROM loader will take a
// maximum of ???.
const SRAM_SIZE: usize = 0x20_0000;

const ARCH: &str = "riscv64";
const TARGET: &str = "riscv64imac-unknown-none-elf";

const BT0_BIN: &str = "starfive-visionfive2-bt0.bin";
const BT0_ELF: &str = "starfive-visionfive2-bt0";

const MAIN_BIN: &str = "starfive-visionfive2-main.bin";
const MAIN_ELF: &str = "starfive-visionfive2-main";

const PAYLOAD_BIN: &str = "starfive-visionfive2-payload.bin";
const PAYLOAD_ELF: &str = "starfive-visionfive2-payload";

const DTFS_DTB: &str = "starfive-visionfive2-dtfs.dtb";
const PAYLOAD_DTB: &str = "starfive-visionfive2-linux.dtb";

const DTFS_IMAGE: &str = "starfive-visionfive2-dtfs.bin";

const IMAGE: &str = "starfive-visionfive2.bin";

pub(crate) fn execute_command(args: &Cli, features: Vec<String>) {
    match args.command {
        Commands::Make => {
            info!("building VisionFive2");
            // Get binutils first so we can fail early
            let binutils_prefix = &find_binutils_prefix_or_fail(ARCH);
            // Build the stages - should we parallelize this?
            xtask_build_jh7110_bt0(&args.env, &features);
            xtask_build_jh7110_main(&args.env);

            objcopy(&args.env, binutils_prefix, TARGET, ARCH, BT0_ELF, BT0_BIN);
            objcopy(&args.env, binutils_prefix, TARGET, ARCH, MAIN_ELF, MAIN_BIN);
            // dtbs
            let dtfs_dts = match env.memory {
                Some(Memory::Nor) => "board.dts",
                _ => {
                    info!("no memory provided, building SRAM image");
                    "sram.dts"
                }
            };
            compile_board_dt(&args.env, TARGET, &board_project_root(), dtfs_dts, DTFS_DTB);
            xtask_copy_dtb(&args.env, TARGET, &board_project_root(), PAYLOAD_DTB);
            // final image
            xtask_build_image(&args.env);
        }
        _ => {
            error!("command {:?} not implemented", args.command);
        }
    }
}

fn xtask_copy_dtb(env: &Env, target: &str, root: &Path, dtb: &str) {
    // TODO: more flexibility; how about non-supervisor payloads, etc?
    if env.supervisor {
        let dtb = env.dtb.as_deref().expect("provide a DTB for LinuxBoot");
        println!("DTB\n  File: {dtb}");
        let dest = dist_dir(env, target).join(PAYLOAD_DTB);
        copy(dtb, dest).expect("failed to copy payload dtb file");
    }
}

fn xtask_build_jh7110_bt0(env: &Env, features: &[String]) {
    trace!("build JH7110 bt0");
    let mut command = get_cargo_cmd_in(env, board_project_root(), "bt0", "build");
    if !features.is_empty() {
        let command_line_features = features.join(",");
        trace!("append command line features: {command_line_features}");
        command.arg("--no-default-features");
        command.args(["--features", &command_line_features]);
    } else {
        trace!("no command line features appended");
    }
    let dram_size = match env.dram_size {
        Some(crate::DramSize::TwoG) => 2,
        Some(crate::DramSize::FourG) => 4,
        Some(crate::DramSize::EightG) => 8,
        None => {
            warn!("no DRAM size provided, falling back to 4G");
            4
        }
    };
    let rustflags_key = "target.riscv64imac-unknown-none-elf.rustflags";
    let rustflags_val = &format!("['--cfg=dram_size=\"{dram_size}G\"']");
    command.args(["--config", &format!("{rustflags_key}={rustflags_val}")]);
    let status = command.status().unwrap();
    trace!("cargo returned {status}");
    if !status.success() {
        error!("cargo build failed with {status}");
        process::exit(1);
    }
}

fn xtask_build_jh7110_main(env: &Env) {
    trace!("build JH7110 main");
    let mut command = get_cargo_cmd_in(env, board_project_root(), "main", "build");
    let status = command.status().unwrap();
    trace!("cargo returned {status}");
    if !status.success() {
        error!("cargo build failed with {status}");
        process::exit(1);
    }
}

fn xtask_build_image(env: &Env) {
    let dir = dist_dir(env, TARGET);
    let dtfs_path = dir.join(DTFS_DTB);
    let dtfs_file = fs::read(dtfs_path).expect("dtfs");
    let dtfs = Fdt::new(&dtfs_file).unwrap();
    let mut areas: Vec<Area> = vec![];
    areas.resize(
        16,
        Area {
            name: "",
            offset: None,
            size: 0,
            file: None,
        },
    );
    let areas = create_areas(&dtfs, &mut areas);

    let dtfs_image_path = dir.join(DTFS_IMAGE);
    if let Err(e) = layout_flash(&dir, &dtfs_image_path, areas.to_vec()) {
        error!("layoutflash fail: {e}");
        process::exit(1);
    }

    // TODO: how else do we do layoutflash + header?
    trace!("add header to {dtfs_image_path:?}");
    let dat = fs::read(dtfs_image_path).expect("DTFS image");

    // NOTE: the header is for what fits in SRAM
    let cut_size = SRAM_SIZE;
    let cut = core::cmp::min(cut_size, dat.len());
    trace!("image size {:08x} cut down to {cut:08x}", dat.len());
    let out = spl_create_hdr(dat[HEADER_SIZE as usize..cut].to_vec());
    trace!("final size {:08x}", out.len());
    let out_path = dir.join(IMAGE);
    fs::write(out_path.clone(), out).expect("writing final image");

    println!("======= DONE =======");
    println!("Output file: {:?}", &out_path.into_os_string());
}

// FIXME: factor out, rework, share!
fn board_project_root() -> std::path::PathBuf {
    project_root().join("src/mainboard/starfive/visionfive2")
}
