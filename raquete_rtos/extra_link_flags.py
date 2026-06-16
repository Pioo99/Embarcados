# O builder cmsis do ststm32 nao propaga build_flags ao linker;
# o gcc precisa do float-abi no link para escolher a multilib correta.
Import("env")
env.Append(LINKFLAGS=["-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"])
