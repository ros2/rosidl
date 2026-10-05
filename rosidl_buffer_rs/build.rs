// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

fn main() {
    #[cfg(not(feature = "use_ros_shim"))]
    {
        let ament_prefix_path = std::env::var_os("AMENT_PREFIX_PATH")
            .expect("AMENT_PREFIX_PATH is not set; source the ROS installation first");
        let prefixes: Vec<_> = std::env::split_paths(&ament_prefix_path).collect();
        let buffer_include = prefixes
            .iter()
            .flat_map(|prefix| [prefix.join("include/rosidl_buffer"), prefix.join("include")])
            .find(|directory| directory.join("rosidl_buffer/buffer.hpp").is_file())
            .expect("rosidl_buffer headers are missing from AMENT_PREFIX_PATH");
        cxx_build::CFG.exported_header_dirs.push(&buffer_include);
        cxx_build::bridge("src/lib.rs")
            .file("src/buffer_bridge.cpp")
            .include(&buffer_include)
            .std("c++20")
            .compile("rosidl_buffer_rs_bridge");

        for library in ["rosidl_buffer", "rosidl_runtime_c"] {
            let library_path = prefixes
                .iter()
                .map(|prefix| prefix.join("lib"))
                .find(|directory| {
                    [
                        format!("lib{library}.so"),
                        format!("lib{library}.dylib"),
                        format!("{library}.lib"),
                    ]
                    .iter()
                    .any(|name| directory.join(name).is_file())
                })
                .unwrap_or_else(|| panic!("{library} is missing from AMENT_PREFIX_PATH"));
            println!("cargo:rustc-link-search=native={}", library_path.display());
            println!("cargo:rustc-link-lib={library}");
        }
        println!("cargo:rerun-if-changed={}", buffer_include.display());
    }

    for file in [
        "src/lib.rs",
        "src/buffer_bridge.cpp",
        "src/buffer_bridge.hpp",
        "build.rs",
    ] {
        println!("cargo:rerun-if-changed={file}");
    }
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
}
