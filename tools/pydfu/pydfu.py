from pynrfjprog import API

api = API.API(‘NRF52’)
api.open()
api.connect_to_emu_without_snr()
api.erase_all()
api.close()