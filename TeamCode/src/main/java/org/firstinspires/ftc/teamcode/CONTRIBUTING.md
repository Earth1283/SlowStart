# Contributing Guidelines

These are kept short & simple because **I KNOW YOU WILL NOT READ THIS**. Prove me wrong.

1. Do not write massive comments in JavaDoc style (/** ... **/) at the top of each file. Future you won't read it, it's mostly useless information, and it breaks `head -n`
2. Respect the kernel. `.../teamcode/kernel/*` exists as a **high level abstraction for a reason**. It's here to keep the code clean, and do the hard work for you.
3. **ABSOLUTELY NO MASOCHISM**. If the kernel has a function, **use the function**. DO NOT WRITE YOUR OWN. ADB is a pain. Use `fast-adb.py`. If you don't have a Python interpreter—fine. Use the `adb` binary.
4. Use your comments conservatively. If you are using AI to write code (which I'm pretty sure that you are), tell them to **NOT NARRATE CODE**. It's unnecessary and a pain to my eyes.
5. [WE DO NOT BREAK USERSPACE](https://lkml.org/lkml/2012/12/23/75). If a change results in user programs breaking, it's a bug in the kernel. We never EVER blame the user programs. How hard can this be to understand?

Also please just don't blatantly `cp`/`mv`/`dd`/`rsync` code from 32008 ok?

--Earth1283, developer of team 32008