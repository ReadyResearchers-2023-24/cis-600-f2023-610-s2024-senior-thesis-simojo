using Dates
using DelimitedFiles

function iso_to_seconds(iso_date::String)
  dt = DateTime(split(iso_date, '.')[1], "yyyy-mm-ddTHH:MM:SS")
  return Dates.datetime2unix(dt)
end

function calculate_deltas(filename::String, column::Int)
  data = readdlm(filename, ' ', String, skipstart=2)
  timestamps = map(row -> iso_to_seconds(row[column]), eachrow(data))
  deltas = diff(timestamps)
  return deltas
end

# Example usage
filename = "/_scratch/comp/SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-21T00:04:46.769488.txt"  # Replace "data.csv" with your file name
column_number = 5  # Replace 3 with the column number you want to extract
deltas = calculate_deltas(filename, column_number)

# write results
io = open("deltas.txt", "w")
i = 0
for delta in deltas
  global i
  write(io, "$i $delta\n")
  i += 1
end
close(io)
println("results written to deltas.txt")
