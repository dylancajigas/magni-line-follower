using BarcodeStandard;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Windows.Forms;

namespace BarcodeGenerator
{
    public partial class Batch : Form
    {
        private int[] imgWidths =
        {
            500,
            1000,
            2000
        };

        private int[] imgHeights =
        {
            120,
            240,
            480
        };

        public Batch()
        {
            InitializeComponent();
            sizePicker.SelectedIndex = 0;
            formatPicker.SelectedIndex = 0;
        }

        private void cancelButton_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void generateButton_Click(object sender, EventArgs e)
        {
            // set up interal variables
            List<string> lengths = new List<string>();
            List<string> stops = new List<string>();

            string lengthText = lengthTextBox.Text;
            string stopsText = stopsTextBox.Text;

            // Analyze text boxes and add entries into lists
            while (lengthText.Length > 0)
            {
                if (lengthText.Contains("\n"))
                {
                    lengths.Add(lengthText.Substring(0, lengthText.IndexOf("\n")).Trim());
                    lengthText = lengthText.Substring(lengthText.IndexOf("\n") + 1);
                }
                else
                {
                    lengths.Add(lengthText.Trim());
                    lengthText = "";
                }
            }
            while (stopsText.Length > 0)
            {
                if (stopsText.Contains("\n"))
                {
                    stops.Add(stopsText.Substring(0, stopsText.IndexOf("\n")).Trim());
                    stopsText = stopsText.Substring(stopsText.IndexOf("\n") + 1);
                }
                else
                {
                    stops.Add(stopsText.Trim());
                    stopsText = "";
                }
            }

            // Error if lines are mismatched
            if(stops.Count != lengths.Count)
            {
                MessageBox.Show("Enter the same number of lengths and stops!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            
            // Cleans and checks each entry
            for(int i = 0; i < lengths.Count; i++)
            {
                if(!int.TryParse(lengths[i], out int curr) || curr <= 0){
                    MessageBox.Show("Length \"" + lengths[i] + "\" is invalid!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }
            }
            for (int i = 0; i < stops.Count; i++)
            {
                var tempElement = stops[i];
                List<int> tempStops = new List<int>();
                int curr = 0;
                while(tempElement.Contains(','))
                {
                    if(tempElement.IndexOf(',') == 0 || 
                        !int.TryParse(tempElement.Substring(0, tempElement.IndexOf(',')).Trim(), out curr) ||
                        tempStops.Contains(curr) ||
                        curr > int.Parse(lengths[i]) ||
                        curr <= 0)
                    {
                        MessageBox.Show("Stop list \"" + stops[i] + "\" is invalid!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        return;
                    }
                    else
                    {
                        tempStops.Add(curr);
                        tempElement = tempElement.Substring(tempElement.IndexOf(",") + 1);
                    }
                }
                if(tempElement.Length == 0 ||
                    !int.TryParse(tempElement.Trim(), out curr) ||
                    tempStops.Contains(curr) ||
                    curr > int.Parse(lengths[i]) ||
                    curr <= 0)
                {
                    MessageBox.Show("Stop list \"" + stops[i] + "\" is invalid!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }
                else
                {
                    tempStops.Add(curr);
                }
                var cleanedStops = "";
                for(int j = 0; j < tempStops.Count; j++)
                {
                    cleanedStops += tempStops[j] + ",";
                }
                cleanedStops = cleanedStops.Substring(0, cleanedStops.Length - 1);
                stops.RemoveAt(i);
                stops.Insert(i, cleanedStops);
            }

            // Generates strings to encode
            List<string> codes = new List<string>();
            for(int i = 0; i < lengths.Count; i++)
            {
                codes.Add("L=" + lengths[i] + ":S=" + stops[i]);
            }

            var barcodeCreator = new Barcode
            {
                IncludeLabel = true
            };

            // Open folder dialog box and set filepath
            string filepath = "";
            FolderBrowserDialog fb = new FolderBrowserDialog();
            if (fb.ShowDialog() == DialogResult.OK)
            {
                //Generate barcode images and save to folder
                filepath = fb.SelectedPath;
                for (int i = 0; i < codes.Count; i++)
                {
                    var fileName = codes[i].Replace('=', '-').Replace(':', '_');
                    if (formatPicker.SelectedItem.Equals(".jpg"))
                    {
                        fileName += ".jpg";
                    }
                    else if (formatPicker.SelectedItem.Equals(".png"))
                    {
                        fileName += ".png";
                    }

                    FileStream tempFs = new FileStream(filepath + "\\" + fileName, FileMode.Create);
                    Image tempBarcode = Image.FromStream(barcodeCreator.Encode(BarcodeStandard.Type.Code128, codes[i], imgWidths[sizePicker.SelectedIndex], imgHeights[sizePicker.SelectedIndex]).Encode().AsStream());
                    if (formatPicker.SelectedItem.Equals(".jpg"))
                    {
                        tempBarcode.Save(tempFs, System.Drawing.Imaging.ImageFormat.Jpeg);
                    }
                    else if (formatPicker.SelectedItem.Equals(".png"))
                    {
                        tempBarcode.Save(tempFs, System.Drawing.Imaging.ImageFormat.Png);
                    }
                    tempFs.Close();
                    tempBarcode.Dispose();
                }

                MessageBox.Show("Barcodes have been saved to " + filepath, "Complete", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            
            this.Close();
        }

        private void graphicalButton_Click(object sender, EventArgs e)
        {
            // open graphical window
            List<string> temp = new List<string> { "", "" };
            Graphical form = new Graphical(temp);
            form.ShowDialog();
            if (temp[0].Length > 0)
            {
                lengthTextBox.Text = temp[0] + "\r\n" + lengthTextBox.Text;
                stopsTextBox.Text = temp[1] + "\r\n" + stopsTextBox.Text;
            }
        }
    }
}
