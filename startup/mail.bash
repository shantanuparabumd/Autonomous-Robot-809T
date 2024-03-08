#!/bin/bash

# Set email parameters
recipient="shantanuparab99@gmail.com"
subject="Test Email"
body="This is a test email sent from Bash script."

# Send the email
echo "$body" | mail -s "$subject" "$recipient"

echo "Email sent successfully."
