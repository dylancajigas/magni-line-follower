using BarcodeStandard;
using SkiaSharp;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Windows.Forms;

namespace BarcodeGenerator
{
    public partial class Default : Form
    {
        // internal variables
        private Image barcodeImage;
        private int size; // 0 - small, 1 - medium, 2 - large

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



        public Default()
        {
            InitializeComponent();
            barcodeImage = null;
            size = 0;
            sizePicker.SelectedIndex = 0;
        }

        private void generateButton_Click(object sender, EventArgs e)
        {
            // check if text box entries are valid/parse length box
            if (int.TryParse(lengthTextBox.Text, out var numStops) && numStops >= 0)
            {
                // create string for encoding
                var barcodeText = "L=" + lengthTextBox.Text;

                var tempStops = stopsTextBox.Text;
                var stopsList = new List<int>();
                var currStop = -1;

                // parse stops box
                while (tempStops.Contains(','))
                { 
                    // check if each stop is valid, add them to a list
                    if (tempStops.IndexOf(',') == 0 || 
                        !int.TryParse(tempStops.Substring(0, tempStops.IndexOf(',')).Trim(), out currStop) || 
                        stopsList.Contains(currStop) || 
                        currStop > numStops)
                    {
                        MessageBox.Show("Enter a valid list of stops!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        return;
                    }
                    else if (currStop > numStops)
                    {
                        MessageBox.Show("Stop out of range!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        return;
                    }
                    stopsList.Add(currStop);
                    tempStops = tempStops.Substring(tempStops.IndexOf(',') + 1);
                }
                if(!int.TryParse(tempStops, out currStop) || stopsList.Contains(currStop) || currStop > numStops || currStop <= 0)
                {
                    MessageBox.Show("Enter a valid list of stops!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }
                stopsList.Add(currStop);

                // sort stops to lowest to highest
                stopsList.Sort();

                // update string for encoding
                barcodeText += ":S=";
                for (int i = 0; i < stopsList.Count; i++)
                {
                    barcodeText += stopsList[i].ToString() + ",";
                }
                barcodeText = barcodeText.Substring(0, barcodeText.Length - 1);

                // create barcode
                using (Barcode barcodeCreator = new Barcode { IncludeLabel = true }) 
                {
                    try
                    {
                        // generate barcode
                        using (var img = barcodeCreator.Encode(BarcodeStandard.Type.Code128, barcodeText, imgWidths[size], imgHeights[size]))
                        {
                            barcodeImage = Image.FromStream(img.Encode().AsStream());

                            //resize image for preview
                            using (var surface = SKSurface.Create(new SKImageInfo(imgWidths[0], imgHeights[0],
                                SKImageInfo.PlatformColorType, SKAlphaType.Premul)))
                            using (var paint = new SKPaint())
                            {
                                // high quality with antialiasing
                                paint.IsAntialias = true;
                                paint.FilterQuality = SKFilterQuality.High;

                                // draw the bitmap to fill the surface
                                surface.Canvas.DrawImage(img, new SKRectI(0, 0, imgWidths[0], imgHeights[0]),
                                    paint);
                                surface.Canvas.Flush();

                                using (var newImg = surface.Snapshot())
                                {
                                    barcodePictureBox.Image = Image.FromStream(newImg.Encode().AsStream());
                                }
                            }
                        }
                        saveButton.Enabled = true;
                    }
                    catch (Exception _)
                    {
                        MessageBox.Show("Message is too large for image size!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        return;
                    }
                }
            } 
            else
            {
                MessageBox.Show("Enter a valid number of stations!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void lengthTextBox_KeyPress(object sender, KeyPressEventArgs e)
        {
            // enter key submits form
            if (e.KeyChar == (char)Keys.Enter)
            {
                generateButton_Click(sender, new EventArgs());
            }
        }

        private void stopsTextBox_KeyPress(object sender, KeyPressEventArgs e)
        {
            // enter key submits form
            if (e.KeyChar == (char)Keys.Enter)
            {
                generateButton_Click(sender, new EventArgs());
            }
        }

        private void saveButton_Click(object sender, EventArgs e)
        {
            // open file selection dialogue, saves barcode to file
            SaveFileDialog saveFileDialog = new SaveFileDialog();
            saveFileDialog.Filter = "JPEG Image|*.jpg|PNG Image|*.bmp";
            saveFileDialog.Title = "Save an Image File";
            if (saveFileDialog.ShowDialog() == DialogResult.OK)
            {
                if (saveFileDialog.FileName != "")
                {
                    FileStream fs = (FileStream)saveFileDialog.OpenFile();
                    Console.WriteLine(fs.Name);
                    switch (saveFileDialog.FilterIndex)
                    {
                        case 1:
                            barcodeImage.Save(fs, System.Drawing.Imaging.ImageFormat.Jpeg);
                            break;
                        case 2:
                            barcodeImage.Save(fs, System.Drawing.Imaging.ImageFormat.Png);
                            break;
                    }
                    fs.Close();
                }
            }
        }

        private void batchButton_Click(object sender, EventArgs e)
        {
            // open batch creation form
            Batch form = new Batch();
            form.ShowDialog();
        }

        private void graphicalButton_Click(object sender, EventArgs e)
        {
            // open graphical form
            List<string> temp = new List<string> { "", "" };
            Graphical form = new Graphical(temp);
            form.ShowDialog();
            lengthTextBox.Text = temp[0];
            stopsTextBox.Text = temp[1];
        }

        private void sizePicker_SelectedIndexChanged(object sender, EventArgs e)
        {
            // change size
            size = sizePicker.SelectedIndex;
        }
    }
}
