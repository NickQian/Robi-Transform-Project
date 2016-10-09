#Header    header
uint8     capacity                          # 1-100 percentage

#float32  voltage
#float32  current
#float32  charge                            #in Ah
#float32  capacity
#float32  design_capacity
#float32  percentage
#uint8    power_supply_status
#uint8    power_supply_health
#uint8    power_supply_technology
#bool     present

#float32[]  cell_voltage    #

#string     location
#string     serial_number

#-------------- Power supply status constants -------------------
uint8 POWER_SUPPLY_STATUS_UNKNOWN     = 0
uint8 POWER_SUPPLY_STATUS_CHARGING    = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING= 3
uint8 POWER_SUPPLY_STSTUS_FULL        = 4

#----------------------- Health ---------------------------------
uint8 POWER_SUPPLY_HEALTH_UNKNOWN     = 0
uint8 POWER_SUPPLY_HEALTH_GOOD        = 1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT    = 2
uint8 POWER_SUPPLY_HEALTH_DEAD        = 3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
