namespace BarcodeGenerator
{
    partial class Default
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Default));
            this.lengthLabel = new System.Windows.Forms.Label();
            this.lengthTextBox = new System.Windows.Forms.TextBox();
            this.generateButton = new System.Windows.Forms.Button();
            this.stopsLabel = new System.Windows.Forms.Label();
            this.stopsTextBox = new System.Windows.Forms.TextBox();
            this.barcodePictureBox = new System.Windows.Forms.PictureBox();
            this.barcodeLabel = new System.Windows.Forms.Label();
            this.saveButton = new System.Windows.Forms.Button();
            this.batchButton = new System.Windows.Forms.Button();
            this.graphicalButton = new System.Windows.Forms.Button();
            this.sizePicker = new System.Windows.Forms.ComboBox();
            this.label1 = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.barcodePictureBox)).BeginInit();
            this.SuspendLayout();
            // 
            // lengthLabel
            // 
            this.lengthLabel.AutoSize = true;
            this.lengthLabel.Location = new System.Drawing.Point(27, 26);
            this.lengthLabel.Name = "lengthLabel";
            this.lengthLabel.Size = new System.Drawing.Size(306, 16);
            this.lengthLabel.TabIndex = 0;
            this.lengthLabel.Text = "Number of stations in the track (not including home)";
            // 
            // lengthTextBox
            // 
            this.lengthTextBox.Location = new System.Drawing.Point(30, 45);
            this.lengthTextBox.Name = "lengthTextBox";
            this.lengthTextBox.Size = new System.Drawing.Size(100, 22);
            this.lengthTextBox.TabIndex = 0;
            this.lengthTextBox.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.lengthTextBox_KeyPress);
            // 
            // generateButton
            // 
            this.generateButton.Location = new System.Drawing.Point(560, 153);
            this.generateButton.Name = "generateButton";
            this.generateButton.Size = new System.Drawing.Size(82, 25);
            this.generateButton.TabIndex = 2;
            this.generateButton.Text = "Generate";
            this.generateButton.UseVisualStyleBackColor = true;
            this.generateButton.Click += new System.EventHandler(this.generateButton_Click);
            // 
            // stopsLabel
            // 
            this.stopsLabel.AutoSize = true;
            this.stopsLabel.Location = new System.Drawing.Point(27, 80);
            this.stopsLabel.Name = "stopsLabel";
            this.stopsLabel.Size = new System.Drawing.Size(282, 16);
            this.stopsLabel.TabIndex = 3;
            this.stopsLabel.Text = "Stations to stop at (as a comma-separated list)";
            // 
            // stopsTextBox
            // 
            this.stopsTextBox.Location = new System.Drawing.Point(30, 103);
            this.stopsTextBox.Name = "stopsTextBox";
            this.stopsTextBox.Size = new System.Drawing.Size(446, 22);
            this.stopsTextBox.TabIndex = 1;
            this.stopsTextBox.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.stopsTextBox_KeyPress);
            // 
            // barcodePictureBox
            // 
            this.barcodePictureBox.BackColor = System.Drawing.Color.White;
            this.barcodePictureBox.Location = new System.Drawing.Point(11, 217);
            this.barcodePictureBox.Name = "barcodePictureBox";
            this.barcodePictureBox.Size = new System.Drawing.Size(631, 150);
            this.barcodePictureBox.TabIndex = 5;
            this.barcodePictureBox.TabStop = false;
            // 
            // barcodeLabel
            // 
            this.barcodeLabel.AutoSize = true;
            this.barcodeLabel.Location = new System.Drawing.Point(27, 198);
            this.barcodeLabel.Name = "barcodeLabel";
            this.barcodeLabel.Size = new System.Drawing.Size(58, 16);
            this.barcodeLabel.TabIndex = 6;
            this.barcodeLabel.Text = "Preview:";
            // 
            // saveButton
            // 
            this.saveButton.Enabled = false;
            this.saveButton.Location = new System.Drawing.Point(567, 373);
            this.saveButton.Name = "saveButton";
            this.saveButton.Size = new System.Drawing.Size(75, 24);
            this.saveButton.TabIndex = 3;
            this.saveButton.Text = "Save As";
            this.saveButton.UseVisualStyleBackColor = true;
            this.saveButton.Click += new System.EventHandler(this.saveButton_Click);
            // 
            // batchButton
            // 
            this.batchButton.Location = new System.Drawing.Point(504, 47);
            this.batchButton.Name = "batchButton";
            this.batchButton.Size = new System.Drawing.Size(138, 29);
            this.batchButton.TabIndex = 8;
            this.batchButton.TabStop = false;
            this.batchButton.Text = "Batch creation";
            this.batchButton.UseVisualStyleBackColor = true;
            this.batchButton.Click += new System.EventHandler(this.batchButton_Click);
            // 
            // graphicalButton
            // 
            this.graphicalButton.Location = new System.Drawing.Point(504, 12);
            this.graphicalButton.Name = "graphicalButton";
            this.graphicalButton.Size = new System.Drawing.Size(138, 29);
            this.graphicalButton.TabIndex = 8;
            this.graphicalButton.TabStop = false;
            this.graphicalButton.Text = "Graphical Interface";
            this.graphicalButton.UseVisualStyleBackColor = true;
            this.graphicalButton.Click += new System.EventHandler(this.graphicalButton_Click);
            // 
            // sizePicker
            // 
            this.sizePicker.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.sizePicker.FormattingEnabled = true;
            this.sizePicker.Items.AddRange(new object[] {
            "Small (500x120)",
            "Medium (1000x240)",
            "Large (2000x480)"});
            this.sizePicker.Location = new System.Drawing.Point(30, 153);
            this.sizePicker.Name = "sizePicker";
            this.sizePicker.Size = new System.Drawing.Size(155, 24);
            this.sizePicker.TabIndex = 9;
            this.sizePicker.SelectedIndexChanged += new System.EventHandler(this.sizePicker_SelectedIndexChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(27, 133);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(72, 16);
            this.label1.TabIndex = 3;
            this.label1.Text = "Image size";
            // 
            // Default
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(654, 409);
            this.Controls.Add(this.sizePicker);
            this.Controls.Add(this.graphicalButton);
            this.Controls.Add(this.batchButton);
            this.Controls.Add(this.saveButton);
            this.Controls.Add(this.barcodeLabel);
            this.Controls.Add(this.barcodePictureBox);
            this.Controls.Add(this.stopsTextBox);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.stopsLabel);
            this.Controls.Add(this.generateButton);
            this.Controls.Add(this.lengthTextBox);
            this.Controls.Add(this.lengthLabel);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "Default";
            this.Text = "Barcode Generator";
            ((System.ComponentModel.ISupportInitialize)(this.barcodePictureBox)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label lengthLabel;
        private System.Windows.Forms.TextBox lengthTextBox;
        private System.Windows.Forms.Button generateButton;
        private System.Windows.Forms.Label stopsLabel;
        private System.Windows.Forms.TextBox stopsTextBox;
        private System.Windows.Forms.PictureBox barcodePictureBox;
        private System.Windows.Forms.Label barcodeLabel;
        private System.Windows.Forms.Button saveButton;
        private System.Windows.Forms.Button batchButton;
        private System.Windows.Forms.Button graphicalButton;
        private System.Windows.Forms.ComboBox sizePicker;
        private System.Windows.Forms.Label label1;
    }
}

