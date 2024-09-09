namespace BarcodeGenerator
{
    partial class Batch
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
            this.fileSystemWatcher1 = new System.IO.FileSystemWatcher();
            this.cancelButton = new System.Windows.Forms.Button();
            this.generateButton = new System.Windows.Forms.Button();
            this.promptLabel = new System.Windows.Forms.Label();
            this.lengthTextBox = new System.Windows.Forms.TextBox();
            this.stopsTextBox = new System.Windows.Forms.TextBox();
            this.lengthLabel = new System.Windows.Forms.Label();
            this.stopsLabel = new System.Windows.Forms.Label();
            this.formatPicker = new System.Windows.Forms.ComboBox();
            this.formatLabel = new System.Windows.Forms.Label();
            this.graphicalButton = new System.Windows.Forms.Button();
            this.sizePicker = new System.Windows.Forms.ComboBox();
            this.sizeLabel = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.fileSystemWatcher1)).BeginInit();
            this.SuspendLayout();
            // 
            // fileSystemWatcher1
            // 
            this.fileSystemWatcher1.EnableRaisingEvents = true;
            this.fileSystemWatcher1.SynchronizingObject = this;
            // 
            // cancelButton
            // 
            this.cancelButton.Location = new System.Drawing.Point(12, 411);
            this.cancelButton.Name = "cancelButton";
            this.cancelButton.Size = new System.Drawing.Size(75, 23);
            this.cancelButton.TabIndex = 3;
            this.cancelButton.Text = "Cancel";
            this.cancelButton.UseVisualStyleBackColor = true;
            this.cancelButton.Click += new System.EventHandler(this.cancelButton_Click);
            // 
            // generateButton
            // 
            this.generateButton.Location = new System.Drawing.Point(462, 410);
            this.generateButton.Name = "generateButton";
            this.generateButton.Size = new System.Drawing.Size(86, 23);
            this.generateButton.TabIndex = 6;
            this.generateButton.Text = "Generate";
            this.generateButton.UseVisualStyleBackColor = true;
            this.generateButton.Click += new System.EventHandler(this.generateButton_Click);
            // 
            // promptLabel
            // 
            this.promptLabel.AutoSize = true;
            this.promptLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.promptLabel.Location = new System.Drawing.Point(11, 9);
            this.promptLabel.Name = "promptLabel";
            this.promptLabel.Size = new System.Drawing.Size(345, 18);
            this.promptLabel.TabIndex = 2;
            this.promptLabel.Text = "Batch Creation: Enter desired info on separate lines";
            // 
            // lengthTextBox
            // 
            this.lengthTextBox.Location = new System.Drawing.Point(12, 75);
            this.lengthTextBox.Multiline = true;
            this.lengthTextBox.Name = "lengthTextBox";
            this.lengthTextBox.Size = new System.Drawing.Size(107, 303);
            this.lengthTextBox.TabIndex = 1;
            // 
            // stopsTextBox
            // 
            this.stopsTextBox.Location = new System.Drawing.Point(134, 75);
            this.stopsTextBox.Multiline = true;
            this.stopsTextBox.Name = "stopsTextBox";
            this.stopsTextBox.Size = new System.Drawing.Size(413, 303);
            this.stopsTextBox.TabIndex = 2;
            // 
            // lengthLabel
            // 
            this.lengthLabel.AutoSize = true;
            this.lengthLabel.Location = new System.Drawing.Point(12, 51);
            this.lengthLabel.Name = "lengthLabel";
            this.lengthLabel.Size = new System.Drawing.Size(105, 16);
            this.lengthLabel.TabIndex = 4;
            this.lengthLabel.Text = "Number of stops";
            // 
            // stopsLabel
            // 
            this.stopsLabel.AutoSize = true;
            this.stopsLabel.Location = new System.Drawing.Point(136, 51);
            this.stopsLabel.Name = "stopsLabel";
            this.stopsLabel.Size = new System.Drawing.Size(314, 16);
            this.stopsLabel.TabIndex = 5;
            this.stopsLabel.Text = "Stations to stop at (enter as a comma separated list)";
            // 
            // formatPicker
            // 
            this.formatPicker.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.formatPicker.FormattingEnabled = true;
            this.formatPicker.Items.AddRange(new object[] {
            ".jpg",
            ".png"});
            this.formatPicker.Location = new System.Drawing.Point(368, 409);
            this.formatPicker.Name = "formatPicker";
            this.formatPicker.Size = new System.Drawing.Size(88, 24);
            this.formatPicker.TabIndex = 5;
            // 
            // formatLabel
            // 
            this.formatLabel.AutoSize = true;
            this.formatLabel.Location = new System.Drawing.Point(366, 390);
            this.formatLabel.Name = "formatLabel";
            this.formatLabel.Size = new System.Drawing.Size(88, 16);
            this.formatLabel.TabIndex = 10;
            this.formatLabel.Text = "Image format:";
            // 
            // graphicalButton
            // 
            this.graphicalButton.Location = new System.Drawing.Point(466, 5);
            this.graphicalButton.Name = "graphicalButton";
            this.graphicalButton.Size = new System.Drawing.Size(80, 28);
            this.graphicalButton.TabIndex = 7;
            this.graphicalButton.Text = "Graphical";
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
            this.sizePicker.Location = new System.Drawing.Point(218, 409);
            this.sizePicker.Name = "sizePicker";
            this.sizePicker.Size = new System.Drawing.Size(136, 24);
            this.sizePicker.TabIndex = 4;
            // 
            // sizeLabel
            // 
            this.sizeLabel.AutoSize = true;
            this.sizeLabel.Location = new System.Drawing.Point(217, 390);
            this.sizeLabel.Name = "sizeLabel";
            this.sizeLabel.Size = new System.Drawing.Size(75, 16);
            this.sizeLabel.TabIndex = 10;
            this.sizeLabel.Text = "Image size:";
            // 
            // Batch
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(575, 476);
            this.ControlBox = false;
            this.Controls.Add(this.graphicalButton);
            this.Controls.Add(this.sizeLabel);
            this.Controls.Add(this.formatLabel);
            this.Controls.Add(this.sizePicker);
            this.Controls.Add(this.formatPicker);
            this.Controls.Add(this.stopsLabel);
            this.Controls.Add(this.lengthLabel);
            this.Controls.Add(this.stopsTextBox);
            this.Controls.Add(this.lengthTextBox);
            this.Controls.Add(this.promptLabel);
            this.Controls.Add(this.generateButton);
            this.Controls.Add(this.cancelButton);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
            this.Name = "Batch";
            this.Text = "Batch Create";
            ((System.ComponentModel.ISupportInitialize)(this.fileSystemWatcher1)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.IO.FileSystemWatcher fileSystemWatcher1;
        private System.Windows.Forms.Button cancelButton;
        private System.Windows.Forms.Label promptLabel;
        private System.Windows.Forms.Button generateButton;
        private System.Windows.Forms.TextBox stopsTextBox;
        private System.Windows.Forms.TextBox lengthTextBox;
        private System.Windows.Forms.Label lengthLabel;
        private System.Windows.Forms.Label stopsLabel;
        private System.Windows.Forms.ComboBox formatPicker;
        private System.Windows.Forms.Label formatLabel;
        private System.Windows.Forms.Button graphicalButton;
        private System.Windows.Forms.Label sizeLabel;
        private System.Windows.Forms.ComboBox sizePicker;
    }
}