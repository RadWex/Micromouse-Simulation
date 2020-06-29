import sim


class Communication(object):
    def __init__(self):
        self.simulation = sim.simxStart(
            '127.0.0.1', 19999, True, True, 5000, 5)
        assert self.simulation != -1
        print("Uchwyt połączenia: %d" % self.simulation)

    def start(self):
        # Zatrzymaj symulację, jeżeli jest uruchomiona
        result = sim.simxStopSimulation(
            self.simulation, sim.simx_opmode_blocking)
        assert result == sim.simx_return_ok

        result = sim.simxStartSimulation(
            self.simulation, sim.simx_opmode_blocking)
        assert result == sim.simx_return_ok
