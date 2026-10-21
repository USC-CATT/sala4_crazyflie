# Zach's Notes

## Fri Jun 5
Installing tailscale, per Andrew's Instructions.


> * [Tailscale 
>   login](https://login.tailscale.com/admin/invite/F6Agjhhyom3hY9uADvqs11) this 
>   is how you can get vpn access to the computer just create an account
> * i would then test you can ssh first into the machine  \
> * then after you can successfully ssh you can use rdp if you use windows to 
>   connect to the machine \ 
> * please let me know if you use another operating system as it will work 
>   differently \
> * for MacOs, the program is likely Screensharing
> 
> its a little slow outside of a loaded application i will work on a better 
> connection because it was not this slow when i was closer

Installed by downloading from the website (not from the app store, gave it all 
requested permissions. Made my account using my usc email in the hopes it would 
just work.

Press connect button on tailscale app to attempt to conenct to network. Login 
using gmail and your usc account and it should connect just fine to the usc.edu 
netowrk.

Edit 9/30/2026 ZRWH: Looks like you have to use your personal email, if you use 
your usc email it automatically c onnects you to the USC tailnet, which you are 
not an admin of.

You need to be invited to the tailnet, so once you are you should be able to get 
the ip address by looking at your tailnet list of machines. Then you can try to 
ssh into the machine using its ip address, which you should be able to do since 
you are on the tailnet


## Mon Sep 21 2026

Relearning how to use the crazyflie

bingtao and noah are changing the firmware a lot on CF 2, so we're using CF#1 
for our testing

got it to crash successfully, on both machines. which is cool.

need to get andrew to write up notes on what was run to get the thing to run
