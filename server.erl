-module(server).
-compile(export_all).

start() -> register(controller, spawn(fun() -> init() end)).

init() -> io:format("my pid is ~p ~n", [self()]), control().

control() ->
    receive
        {ciao} -> io:format("Ho ricevuto un mexy,io");
        _ -> io:format("Received something else...")
    end.
