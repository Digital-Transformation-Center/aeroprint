from starling.helix import Helix

def main(args=None):
    helix = Helix(20)
    helix.set_params(1, 1, 0.5)
    print(helix.get_state(600).z)
    state = helix.get_state(600)
    print("Helix; x, y, z:", state.x, state.y, state.z)