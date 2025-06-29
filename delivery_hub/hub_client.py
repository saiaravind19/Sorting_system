from pymodbus.client import ModbusTcpClient
import time

SLOT_INDEX = 2            # Slot 3 (0-indexed)
TRIGGER_COIL = SLOT_INDEX
SLOT_STATE_BASE = 10      # HR[10–15]
SLOT_COUNT_REG = 0        # HR[0]
PACKAGE_COUNT_REG = 1     # HR[1]
NUM_SLOTS = 6

client = ModbusTcpClient("localhost", port=5020)
client.connect()

print(f"[Robot] 🚚 Delivering to slot {SLOT_INDEX + 1}...")
#client.write_coil(1, 1)
#client.write_coil(2, 1)
#client.write_coil(3, 1)
#client.write_coil(4, 1)
responce = client.write_coil(5, 1)
print(responce)

result = client.read_coils(0, 6, unit=0)
print(result.bits) # modbus packs the entire data in (N+7)/8) round down to nearest integer

#client.write_coil(0, 1)

time.sleep(2)
#client.write_coil(TRIGGER_COIL, False)

# Slot status
responce = client.read_holding_registers(0)

data = responce.registers[0]

print("\n[Robot] 📦 Slot States:",data)
#for i, s in enumerate(slots.registers):
#    print(f"  Slot {i+1}: {'Occupied' if s else 'Free'}")

# Package count and free slots
#counters = client.read_holding_registers(SLOT_COUNT_REG, 2)
#print(f"\n[Robot] 📊 Slots Available: {counters.registers[0]}")
#print(f"[Robot] 📈 Packages Processed: {counters.registers[1]}")

client.close()
