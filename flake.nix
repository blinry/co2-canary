{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    esp32 = {
      url = "github:knarkzel/esp32";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = {
    self,
    nixpkgs,
    esp32,
  }: let
    pkgs = import nixpkgs {system = "x86_64-linux";};
    idf-rust = esp32.packages.x86_64-linux.esp32;
  in {
    devShells.x86_64-linux.default = pkgs.mkShell {
      buildInputs = [
        idf-rust
        pkgs.cargo-espflash
      ];

      shellHook = ''
        export PATH="${idf-rust}/.rustup/toolchains/esp/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/bin:$PATH"
        export PATH="${idf-rust}/.rustup/toolchains/esp/bin:$PATH"
        export RUST_SRC_PATH="$(rustc --print sysroot)/lib/rustlib/src/rust/src"
      '';
    };
  };
}
