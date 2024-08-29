import sys
sys.path.append('/home/apeters/project/env/src')

from Functions import Earnings 
z = Earnings.calculate_earnings()
print(z)