import pandas as pd
import os

def optimize_cuts(csv_file_path, output_txt_path="cut_list_output.txt"):
    # --- 1. Constants & Unit Conversions ---
    STOCK_LENGTH_MM = 6 * 12 * 25.4  # 6 feet -> 96 inches -> mm (2436.4 mm)
    KERF_MM = (1/4) * 25.4          # 3/16 inch -> mm (4.7625 mm)

    # --- 2. Load and Clean the Data ---
    print(f"Loading data from {csv_file_path}...")
    df = pd.read_csv(csv_file_path)
    
    items = []
    
    for index, row in df.iterrows():
        item_id = row['Item']
        qty = int(row['Quantity'])
        raw_name = str(row['Name']).strip()
        
        # Check for 'mm' in the string (case-insensitive). If missing, skip silently.
        if 'mm' not in raw_name.lower():
            continue
            
        # Remove 'mm' and any accidental spaces, then convert to float
        clean_str = raw_name.lower().replace('mm', '').replace(' ', '')
        try:
            length = float(clean_str)
        except ValueError:
            continue

        # Add the item to our list 'qty' number of times
        for i in range(qty):
            items.append({
                'label': f"Item {item_id} ({raw_name})",
                'length': length
            })

    # Sort items by length in DESCENDING order (First Fit Decreasing heuristic)
    items.sort(key=lambda x: x['length'], reverse=True)

    # --- 3. Bin Packing Algorithm ---
    bars = []  # List of purchased 6ft bars

    for item in items:
        label = item['label']
        length = item['length']

        if length > STOCK_LENGTH_MM:
            continue # Silently skip items that are too long

        placed = False
        
        # Try to fit the item into an existing bar
        for bar in bars:
            space_needed = length + (KERF_MM if len(bar['cuts']) > 0 else 0)

            if bar['remaining_capacity'] >= space_needed:
                bar['cuts'].append({'label': label, 'length': length})
                bar['remaining_capacity'] -= space_needed
                placed = True
                break

        # If it didn't fit, buy a new bar
        if not placed:
            bars.append({
                'cuts': [{'label': label, 'length': length}],
                'remaining_capacity': STOCK_LENGTH_MM - length
            })

    # --- 4. Console Summary & File Generation ---
    # Print the summary to the console
    print("\n" + "="*50)
    print(f" TOTAL 6-FT BARS TO PURCHASE: {len(bars)}")
    print("="*50)
    print(f"Saving detailed cut list to: '{output_txt_path}'...\n")

    # Prepare the lines of text for the output file
    output_lines = []
    output_lines.append("="*50)
    output_lines.append(f" TOTAL 6-FT BARS TO PURCHASE: {len(bars)}")
    output_lines.append("="*50 + "\n")

    for i, bar in enumerate(bars):
        output_lines.append(f"--- Bar {i + 1} ---")
        for cut in bar['cuts']:
            output_lines.append(f"  [Cut] {cut['length']:>7.2f} mm  ->  Part: {cut['label']}")
        
        # Calculate kerf waste for this specific bar
        kerf_waste = (len(bar['cuts']) - 1) * KERF_MM if len(bar['cuts']) > 0 else 0
        
        output_lines.append("-" * 35)
        output_lines.append(f"  Waste to Kerf: {kerf_waste:.2f} mm")
        output_lines.append(f"  Leftover Scrap: {bar['remaining_capacity']:.2f} mm\n")

    # Write everything to the text file
    with open(output_txt_path, 'w') as f:
        f.write('\n'.join(output_lines))
        
    print("Done! You can now open the text file to see your cuts.")

# ==========================================
# Script Execution & Mock Data Generation
# ==========================================
if __name__ == "__main__":
    file_name = "BOM of Assembly 1.csv"
    output_file = "cut_list_output.txt"
    
    # Generate a dummy file if one doesn't exist
    if not os.path.exists(file_name):
        print(f"File '{file_name}' not found. Generating sample file...")
        sample_data = {
            'Item': [1, 2, 3, 4, 5],
            'Quantity': [1, 4, 2, 1, 1],
            'Name': ['100.96mm', '1200 mm', '850.5', '2500mm', 'Junk Text'] 
        }
        pd.DataFrame(sample_data).to_csv(file_name, index=False)
    
    optimize_cuts(file_name, output_file)