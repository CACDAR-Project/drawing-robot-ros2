# shell.nix
{ pkgs ? import <nixpkgs> {} }:
let
  my-python-packages = p: with p; [
    matplotlib
    numpy
    # other python packages
  ];
  my-python = pkgs.python3.withPackages my-python-packages;
in my-python.env

