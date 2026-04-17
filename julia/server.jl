using Pkg
Pkg.activate(@__DIR__)

include("src/VogelSIM.jl")
using .VogelSIM

start_server()
