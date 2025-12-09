from ps4 import PS4Controller

ps4 = PS4Controller()
ps4.process_joy_ps4(
    axes=[0.2, -0.8, 0.1, -0.4, -1.0, 1.0, 0, 1],
    buttons=[0]*20
)

print(ps4.as_dict())
