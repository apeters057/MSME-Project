def calculate_earnings():
    hourly_rate = float(input("Enter your hourly rate: $"))
    hours_worked = float(input("Enter the number of hours worked: "))
    
    earnings = hourly_rate * hours_worked
    return earnings

# z = calculate_earnings()
# print(z)