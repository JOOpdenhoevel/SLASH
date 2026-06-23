This is the repository for SLASH, a shell and software stack for the
V80 accelerator card. The general structure of the repository and the software
stack are described in the `README.md`.

My current effort is the implementation of a system emulation daemon and the
necessary changes in the software stack for it. The masterplan for this effort
is found in `system_emulation_masterplan.md`. The current state is implementation
step 1: A team of agents has implemented the MVP system emulation daemon in
`slash-emu`, and I'm currently reviewing before moving on to the next steps.