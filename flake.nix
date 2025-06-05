{
  description = "RISCV64 Embedded Toolchain Shell";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs";
  };
  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in {
      devShells.${system}.default = pkgs.mkShell {
        buildInputs = [
          pkgs.pkgsCross.riscv64-embedded.buildPackages.gcc
          pkgs.pkgsCross.riscv64-embedded.buildPackages.gdb
          pkgs.gdb
        ];
        shellHook = ''
          mkdir -p /tmp/riscv-toolchain
          for bin in $(ls $(dirname $(which riscv64-none-elf-gcc))/riscv64-none-elf-*); do
            ln -sf "$bin" "/tmp/riscv-toolchain/$(basename "$bin" | sed 's/riscv64-none-elf-/riscv64-unknown-elf-/')"
          done
          export PATH="/tmp/riscv-toolchain:$PATH"
        '';
      };
    };
}