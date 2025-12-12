import os

jerryExport = input("Enter the full path to your Jerry.io export file: ").strip().strip('"')

if not os.path.exists(jerryExport):
    print(f"Error: File '{jerryExport}' not found!")
    print("Make sure you enter the full path to the file.")
    exit(1)

try:
    with open(jerryExport, 'r') as f:
        lines = f.readlines()
except Exception as e:
    print(f"Error reading file: {e}")
    exit(1)

start_idx = -1
end_idx = -1
for i, line in enumerate(lines):
    if '#PATH-POINTS-START' in line:
        start_idx = i + 1
    elif '#PATH.JERRYIO-DATA' in line:
        end_idx = i
        break

if start_idx == -1 or end_idx == -1:
    print("Error: Could not find path data markers (#PATH-POINTS-START or #PATH.JERRYIO-DATA)")
    print("Make sure you're using a valid Jerry.io export file.")
    exit(1)

points = []
for line in lines[start_idx:end_idx]:
    line = line.strip()                     
    if not line:
        continue
    
    parts = line.split(',')
    if len(parts) < 3:
        continue
    
    x_cm = float(parts[0])
    y_cm = float(parts[1])
    rpm = int(parts[2])
    heading = float(parts[3]) if len(parts) >= 4 else -1
    flag = int(parts[4]) if len(parts) >= 5 else 0  # Optional flag parameter
    
    x_in = x_cm * 0.393701
    y_in = y_cm * 0.393701
    
    speed_percent = int(((rpm - 40) / (600 - 40)) * 100)
    if speed_percent < 0:
        speed_percent = 0
    if speed_percent > 100:
        speed_percent = 100
    
    points.append({
        'x': x_in,
        'y': y_in,
        'speed': speed_percent,
        'heading': heading,
        'flag': flag
    })

output_lines = []
output_lines.append("\nPoint path[] = {")

for i, point in enumerate(points):
    x = point['x']
    y = point['y']
    speed = point['speed']
    heading = point['heading']
    flag = point['flag']
    
    # Build the point string based on which optional parameters are set
    if flag != 0:
        # Flag is set, include everything
        line = f"  {{{x:.3f}, {y:.3f}, {speed}, {heading}, {flag}}}"
    elif heading >= 0:
        # Heading is set but no flag
        line = f"  {{{x:.3f}, {y:.3f}, {speed}, {heading}}}"
    else:
        # Only required parameters
        line = f"  {{{x:.3f}, {y:.3f}, {speed}}}"
    
    if i < len(points) - 1:
        line += ","
    
    output_lines.append(line)

output_lines.append("};")
output_lines.append(f"\n// Path has {len(points)} points")

for line in output_lines:
    print(line)

input_filename = os.path.basename(jerryExport) 
input_name = os.path.splitext(input_filename)[0]

output_dir = r"C:\Users\jokin\Documents\vex-vscode-projects\80550Pirates"
output_file = os.path.join(output_dir, f"{input_name}Formatted.txt")

try:
    with open(output_file, 'w') as f:
        f.write('\n'.join(output_lines))
    print(f"\n✓ Saved to {output_file}")
except Exception as e:
    print(f"\nError saving file: {e}")
